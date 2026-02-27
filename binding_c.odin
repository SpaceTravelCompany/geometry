package geometry

import "base:runtime"
import "core:mem"
import "core:slice"
import "core:c"
import "core:math/linalg"
import "core:math/fixed"

@(private="file") _c_allocator: runtime.Allocator = {
	procedure = c_allocator_proc,
	data = nil
}

@(private="file") alloc_func : proc "c" (size:c.size_t) -> rawptr
@(private="file") calloc_func : proc "c" (nmemb, size:c.size_t) -> rawptr
@(private="file") realloc_func : proc "c" (old:rawptr, size:c.size_t) -> rawptr
@(private="file") free_func : proc "c" (data:rawptr)

@(private="file") c_allocator_proc :: proc(allocator_data: rawptr, mode: runtime.Allocator_Mode,
	size, alignment: int,
	old_memory: rawptr, old_size: int,
	location: runtime.Source_Code_Location = #caller_location) -> ([]byte, runtime.Allocator_Error) {
	switch mode {
	case .Alloc:
		data := calloc_func(1, auto_cast size)
		if data == nil do return nil, .Out_Of_Memory
		return slice.bytes_from_ptr(data, size), nil

	case .Alloc_Non_Zeroed:
		data := alloc_func(auto_cast size)
		if data == nil do return nil, .Out_Of_Memory
		return slice.bytes_from_ptr(data, size), nil

	case .Free:
		free_func(old_memory)

	case .Free_All:
		return nil, .Mode_Not_Implemented

	case .Resize:
		return nil, .Mode_Not_Implemented

	case .Resize_Non_Zeroed:
		data := realloc_func(old_memory, auto_cast size)
		if data == nil do return nil, .Out_Of_Memory
		return slice.bytes_from_ptr(data, size), nil

	case .Query_Features:
		set := (^runtime.Allocator_Mode_Set)(old_memory)
		if set != nil {
			set^ = {.Alloc, .Alloc_Non_Zeroed, .Free, .Resize, .Resize_Non_Zeroed, .Query_Features}
		}
		return nil, nil

	case .Query_Info:
		return nil, .Mode_Not_Implemented
	}
	return nil, nil
}

@export geometry_set_alloc_func :: proc "c" (func: proc "c" (size:c.size_t) -> rawptr) {
	alloc_func = func
}
@export geometry_set_calloc_func :: proc "c" (func: proc "c" (nmemb, size:c.size_t) -> rawptr) {
	calloc_func = func
}
@export geometry_set_realloc_func :: proc "c" (func: proc "c" (old:rawptr, size:c.size_t) -> rawptr) {
	realloc_func = func
}
@export geometry_set_free_func :: proc "c" (func: proc "c" (data:rawptr)) {
	free_func = func
}

// Geometry_Slice matches C struct { void* data, ssize_t len } - same layout as runtime.Raw_Slice
Geometry_Slice :: runtime.Raw_Slice

@(private="file")
Geometry_Vertex :: struct #packed {
	pos:  linalg.Vector2f32,
	uvw:  linalg.Vector3f32,
	color: linalg.Vector4f32,
}

@(private="file")
Geometry_RawShape :: struct {
	vertices: Geometry_Slice,
	indices:  Geometry_Slice,
}

@(private="file")
Geometry_Node :: struct {
	pts:           Geometry_Slice,
	curve_pts_ids: Geometry_Slice,
	color:         linalg.Vector4f32,
	stroke_color:  linalg.Vector4f32,
	thickness:     f64,
	is_closed:     c.bool,
}

@(private="file")
Geometry_Shapes :: struct {
	nodes: Geometry_Slice,
}

// Fixed-point types (FIXED_SHIFT=24, 1.0 = 1<<24)
@(private="file")
Geometry_Fixed :: i64

@(private="file")
Geometry_Vec2_Fixed :: struct {
	x: Geometry_Fixed,
	y: Geometry_Fixed,
}

@(private="file")
Geometry_Vertex_Fixed :: struct #packed {
	pos:  [2]Geometry_Fixed,
	uvw:  linalg.Vector3f32,
	color: linalg.Vector4f32,
}

@(private="file")
Geometry_RawShape_Fixed :: struct {
	vertices: Geometry_Slice,
	indices:  Geometry_Slice,
}

@(private="file")
Geometry_Node_Fixed :: struct {
	pts:           Geometry_Slice,
	n_polys:       Geometry_Slice,
	curve_pts_ids: Geometry_Slice,
	color:         linalg.Vector4f32,
	stroke_color:  linalg.Vector4f32,
	thickness:     Geometry_Fixed,
	is_closed:     c.bool,
}

@(private="file")
Geometry_Shapes_Fixed :: struct {
	nodes: Geometry_Slice,
}

// Error codes for C
@(private="file")
Geometry_Error :: enum i32 {
	None = 0,
	IsPointNotLine = 1,
	EmptyPolygon = 2,
	EmptyColor = 3,
	Allocator = 4,
}

@(export)
geometry_shapes_compute_polygon :: proc "c" (
	input: ^Geometry_Shapes,
	out: ^Geometry_RawShape,
) -> Geometry_Error {
	context = {}
	arena: mem.Dynamic_Arena
	mem.dynamic_arena_init(&arena, _c_allocator, _c_allocator)

	context.allocator = _c_allocator
	context.temp_allocator = mem.dynamic_arena_allocator(&arena)
	defer mem.dynamic_arena_destroy(&arena)

	return _geometry_shapes_compute_polygon_impl(input, out)
}

@(private="file")
_geometry_slice_to_odin :: proc(gs: Geometry_Slice, $T: typeid/[]$E) -> T {
	result: T
	(^runtime.Raw_Slice)(&result)^ = gs
	return result
}

@(private="file")
_geometry_shapes_compute_polygon_impl :: proc(input: ^Geometry_Shapes, out: ^Geometry_RawShape) -> Geometry_Error {
	if input == nil || out == nil do return .None

	// Direct cast - Geometry_Slice has same layout as Odin slice, no copy
	poly := shapes{nodes = _geometry_slice_to_odin(input.nodes, []shape_node)}

	res, err := shapes_compute_polygon(poly, _c_allocator)
	if err != nil {
		#partial switch e in err {
		case __shape_error:
			#partial switch e {
			case .IsPointNotLine: return .IsPointNotLine
			case .EmptyPolygon: return .EmptyPolygon
			case .EmptyColor: return .EmptyColor
			}
		case runtime.Allocator_Error:
			return .Allocator
		}
		return .None
	}

	if len(res.vertices) == 0 || len(res.indices) == 0 {
		out.vertices = {}
		out.indices = {}
		return .None
	}

	// Export slice directly - no copy
	out.vertices = (Geometry_Slice)(runtime.Raw_Slice{data = raw_data(res.vertices), len = len(res.vertices)})
	out.indices = (Geometry_Slice)(runtime.Raw_Slice{data = raw_data(res.indices), len = len(res.indices)})

	return .None
}

@(export)
geometry_raw_shape_free :: proc "c" (shape: ^Geometry_RawShape) {
	if shape == nil do return
	context = runtime.Context{allocator = _c_allocator}
	raw := raw_shape{
		vertices = _geometry_slice_to_odin(shape.vertices, []shape_vertex2d),
		indices  = _geometry_slice_to_odin(shape.indices, []u32),
	}
	raw_shape_free(raw, _c_allocator)
}

// Fixed-point helpers: 1.0 = 1<<24
@(export)
geometry_fixed_from_f64 :: proc "c" (v: f64) -> Geometry_Fixed {
	f: FixedDef
	fixed.init_from_f64(&f, v)
	return Geometry_Fixed(f.i)
}

@(export)
geometry_fixed_to_f64 :: proc "c" (v: Geometry_Fixed) -> f64 {
	return fixed.to_f64(FixedDef{i = v})
}

@(export)
geometry_shapes_compute_polygon_fixed :: proc "c" (
	input: ^Geometry_Shapes_Fixed,
	out: ^Geometry_RawShape_Fixed,
) -> Geometry_Error {
	context = {}
	arena: mem.Dynamic_Arena
	mem.dynamic_arena_init(&arena, _c_allocator, _c_allocator)

	context.allocator = _c_allocator
	context.temp_allocator = mem.dynamic_arena_allocator(&arena)
	defer mem.dynamic_arena_destroy(&arena)

	return _geometry_shapes_compute_polygon_fixed_impl(input, out)
}

@(private="file")
_geometry_shapes_compute_polygon_fixed_impl :: proc(input: ^Geometry_Shapes_Fixed, out: ^Geometry_RawShape_Fixed) -> Geometry_Error {
	if input == nil || out == nil do return .None

	// Direct cast - Geometry_Slice has same layout as Odin slice, no copy
	poly := shapesi64{nodes = _geometry_slice_to_odin(input.nodes, []shape_nodei64)}

	res, err := shapes_compute_polygoni64(poly, _c_allocator)
	if err != nil {
		#partial switch e in err {
		case __shape_error:
			#partial switch e {
			case .IsPointNotLine: return .IsPointNotLine
			case .EmptyPolygon: return .EmptyPolygon
			case .EmptyColor: return .EmptyColor
			}
		case runtime.Allocator_Error:
			return .Allocator
		}
		return .None
	}

	if len(res.vertices) == 0 || len(res.indices) == 0 {
		out.vertices = {}
		out.indices = {}
		return .None
	}

	// Export slice directly - no copy
	out.vertices = (Geometry_Slice)(runtime.Raw_Slice{data = raw_data(res.vertices), len = len(res.vertices)})
	out.indices = (Geometry_Slice)(runtime.Raw_Slice{data = raw_data(res.indices), len = len(res.indices)})

	return .None
}

@(export)
geometry_raw_shape_free_fixed :: proc "c" (shape: ^Geometry_RawShape_Fixed) {
	if shape == nil do return
	context = runtime.Context{allocator = _c_allocator}
	raw := raw_shapei64{
		vertices = _geometry_slice_to_odin(shape.vertices, []shape_vertex2di64),
		indices  = _geometry_slice_to_odin(shape.indices, []u32),
	}
	raw_shapei64_free(raw, _c_allocator)
}
