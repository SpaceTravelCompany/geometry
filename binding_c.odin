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

@(private="file")
Geometry_Vertex :: struct {
	pos: linalg.Vector2f32,
	uvw: linalg.Vector3f32,
	color: linalg.Vector4f32,
}

@(private="file")
Geometry_RawShape :: struct {
	vertices: [^]Geometry_Vertex,
	indices: [^]u32,
	vertex_count:c.size_t,
	index_count: c.size_t,
}

@(private="file")
Geometry_Node :: struct {
	pts: [^]linalg.Vector2f32,
	pts_count: c.size_t,
	curve_pts_ids: [^]u32,
	curve_pts_ids_count: c.size_t,
	color: linalg.Vector4f32,
	stroke_color: linalg.Vector4f32,
	thickness: f64,
	is_closed: c.bool,
}

@(private="file")
Geometry_Shapes :: struct {
	nodes: [^]Geometry_Node,
	node_count: c.size_t,
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
Geometry_Vertex_Fixed :: struct {
	pos: [2]Geometry_Fixed,
	uvw: linalg.Vector3f32,
	color: linalg.Vector4f32,
}

@(private="file")
Geometry_RawShape_Fixed :: struct {
	vertices: [^]Geometry_Vertex_Fixed,
	indices: [^]u32,
	vertex_count: c.size_t,
	index_count: c.size_t,
}

@(private="file")
Geometry_Node_Fixed :: struct {
	pts: [^]Geometry_Vec2_Fixed,
	pts_count: c.size_t,
	curve_pts_ids: [^]u32,
	curve_pts_ids_count: c.size_t,
	color: linalg.Vector4f32,
	stroke_color: linalg.Vector4f32,
	thickness: Geometry_Fixed,
	is_closed: c.bool,
}

@(private="file")
Geometry_Shapes_Fixed :: struct {
	nodes: [^]Geometry_Node_Fixed,
	node_count: c.size_t,
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
_geometry_shapes_compute_polygon_impl :: proc(input: ^Geometry_Shapes, out: ^Geometry_RawShape) -> Geometry_Error {
	if input == nil || out == nil do return .None

	poly: shapes
	nodes, nodes_err := make_non_zeroed_slice([]shape_node, input.node_count, context.temp_allocator)
	if nodes_err != nil do return .Allocator
	poly.nodes = nodes

	for i in 0 ..< input.node_count {
		n := &input.nodes[i]
		if n == nil || n.pts == nil do continue

		node: shape_node
		pts, pts_err := make_non_zeroed_slice([]linalg.Vector2f32, n.pts_count, context.temp_allocator)
		if pts_err != nil do return .Allocator
		node.pts = pts
		mem.copy_non_overlapping(raw_data(node.pts), n.pts, int(n.pts_count) * size_of(linalg.Vector2f32))

		if n.curve_pts_ids != nil && n.curve_pts_ids_count > 0 {
			ids, ids_err := make_non_zeroed_slice([]u32, n.curve_pts_ids_count, context.temp_allocator)
			if ids_err != nil do return .Allocator
			node.curve_pts_ids = ids
			mem.copy_non_overlapping(raw_data(node.curve_pts_ids), n.curve_pts_ids, int(n.curve_pts_ids_count) * size_of(u32))
		}

		node.color = n.color
		node.stroke_color = n.stroke_color
		node.thickness = n.thickness
		node.is_closed = n.is_closed

		poly.nodes[i] = node
	}

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

	out.vertex_count = c.size_t(len(res.vertices))
	out.index_count = c.size_t(len(res.indices))

	if len(res.vertices) == 0 || len(res.indices) == 0 {
		out.vertices = nil
		out.indices = nil
		return .None
	}

	// Geometry_Vertex matches shape_vertex2d layout (pos, uvw, color)
	out.vertices = ([^]Geometry_Vertex)(raw_data(res.vertices))
	out.indices = raw_data(res.indices)

	return .None
}

@(export)
geometry_raw_shape_free :: proc "c" (shape: ^Geometry_RawShape) {
	if shape == nil do return
	context = runtime.Context{allocator = _c_allocator }
	raw := raw_shape{
		vertices = ([^]shape_vertex2d)(shape.vertices)[:shape.vertex_count],
		indices  = ([^]u32)(shape.indices)[:shape.index_count],
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

	poly: shapesi64
	nodes, nodes_err := make_non_zeroed_slice([]shape_nodei64, input.node_count, context.temp_allocator)
	if nodes_err != nil do return .Allocator
	poly.nodes = nodes

	for i in 0 ..< input.node_count {
		n := &input.nodes[i]
		if n == nil || n.pts == nil do continue

		node: shape_nodei64
		pts, pts_err := make_non_zeroed_slice([][2]FixedDef, n.pts_count, context.temp_allocator)
		if pts_err != nil do return .Allocator
		node.pts = pts
		mem.copy_non_overlapping(raw_data(node.pts), n.pts, int(n.pts_count) * size_of(Geometry_Vec2_Fixed))

		if n.curve_pts_ids != nil && n.curve_pts_ids_count > 0 {
			ids, ids_err := make_non_zeroed_slice([]u32, n.curve_pts_ids_count, context.temp_allocator)
			if ids_err != nil do return .Allocator
			node.curve_pts_ids = ids
			mem.copy_non_overlapping(raw_data(node.curve_pts_ids), n.curve_pts_ids, int(n.curve_pts_ids_count) * size_of(u32))
		}

		node.color = n.color
		node.stroke_color = n.stroke_color
		node.thickness = FixedDef{i = n.thickness}
		node.is_closed = n.is_closed

		poly.nodes[i] = node
	}

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

	out.vertex_count = c.size_t(len(res.vertices))
	out.index_count = c.size_t(len(res.indices))

	if len(res.vertices) == 0 || len(res.indices) == 0 {
		out.vertices = nil
		out.indices = nil
		return .None
	}

	// Geometry_Vertex_Fixed matches shape_vertex2di64 layout (pos [2]i64, uvw, color)
	out.vertices = ([^]Geometry_Vertex_Fixed)(raw_data(res.vertices))
	out.indices = raw_data(res.indices)

	return .None
}

@(export)
geometry_raw_shape_free_fixed :: proc "c" (shape: ^Geometry_RawShape_Fixed) {
	if shape == nil do return
	context = runtime.Context{allocator = _c_allocator}
	raw := raw_shapei64{
		vertices = ([^]shape_vertex2di64)(shape.vertices)[:shape.vertex_count],
		indices  = ([^]u32)(shape.indices)[:shape.index_count],
	}
	raw_shapei64_free(raw, _c_allocator)
}
