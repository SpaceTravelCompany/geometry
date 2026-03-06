#+feature using-stmt
package geometry

import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:math/linalg"
import "core:mem"

import "core:math/fixed"
import "shared:utils_private/fixed_ex"

import "shared:utils_private"


//Cover MAX 741455 * 741455
FIXED_SHIFT :: 24
FixedDef :: fixed.Fixed(i64, FIXED_SHIFT)
@(private)
Fixed1 :: fixed.Fixed(i64, FIXED_SHIFT) {
	i = 1 << FIXED_SHIFT,
}
@(private)
Fixed2 :: fixed.Fixed(i64, FIXED_SHIFT) {
	i = 2 << FIXED_SHIFT,
}
@(private)
Fixed3 :: fixed.Fixed(i64, FIXED_SHIFT) {
	i = 3 << FIXED_SHIFT,
}
@(private)
Div2Fixed :: fixed.Fixed(i64, FIXED_SHIFT) {
	i = (1 << FIXED_SHIFT) / 2,
}
@(private)
Div3Fixed :: fixed.Fixed(i64, FIXED_SHIFT) {
	i = (1 << FIXED_SHIFT) / 3,
}
@(private)
Div3Mul2Fixed :: fixed.Fixed(i64, FIXED_SHIFT) {
	i = (2 << FIXED_SHIFT) / 3,
}

//use gpu shader std140 layout
shape_vertex2di64 :: struct {
	pos:   [2]FixedDef,
	uvw:   linalg.Vector3f32,
	color: linalg.Vector4f32,
}
//use gpu shader std140 layout
shape_vertex2d :: struct {
	pos:   linalg.Vector2f32,
	uvw:   linalg.Vector3f32,
	color: linalg.Vector4f32,
}

raw_shape :: struct {
	vertices: []shape_vertex2d,
	indices:  []u32,
}

raw_shapei64 :: struct {
	vertices: []shape_vertex2di64,
	indices:  []u32,
}

@(private)
curve_type :: enum {
	Line,
	Unknown,
	Serpentine,
	Loop,
	// LoopReverse,
	Cusp,
	Quadratic,
}

__shape_error :: enum {
	IsPointNotLine,
	EmptyPolygon,
	EmptyColor,
}

__Geometry_Error :: enum {
	TOO_MANY_EDGES,
	NO_PATHS,
}

Geometry_Error :: union #shared_nil {
	__Geometry_Error,
	runtime.Allocator_Error,
}


shape_error :: union #shared_nil {
	__shape_error,
	__Trianguate_Error,
	__Geometry_Error,
	runtime.Allocator_Error,
}

shape_node :: struct {
	pts:           [][]linalg.Vector2f32,
	curve_pts_ids: [][]u32,
	color:         linalg.Vector4f32,
	stroke_color:  linalg.Vector4f32,
	thickness:     f64,
	is_closed:     bool,
}

shapes :: struct {
	nodes: []shape_node,
}

shape_nodei64 :: struct {
	pts:           [][][2]FixedDef,
	curve_pts_ids: [][]u32,
	color:         linalg.Vector4f32,
	stroke_color:  linalg.Vector4f32,
	thickness:     FixedDef,
	is_closed:     bool,
}

shapesi64 :: struct {
	nodes: []shape_nodei64,
}

raw_shape_free :: proc(self: raw_shape, allocator := context.allocator) {
	delete(self.vertices, allocator)
	delete(self.indices, allocator)
}

raw_shapei64_free :: proc(self: raw_shapei64, allocator := context.allocator) {
	delete(self.vertices, allocator)
	delete(self.indices, allocator)
}

raw_shapei64_clone :: proc(
	self: raw_shapei64,
	allocator := context.allocator,
) -> (
	res: raw_shapei64,
	err: runtime.Allocator_Error,
) #optional_allocator_error {
	res.vertices = utils_private.make_non_zeroed_slice(
		[]shape_vertex2di64,
		len(self.vertices),
		allocator,
	) or_return
	defer if err != nil do delete(res.vertices, allocator)

	res.indices = utils_private.make_non_zeroed_slice(
		[]u32,
		len(self.indices),
		allocator,
	) or_return

	intrinsics.mem_copy_non_overlapping(
		&res.vertices[0],
		&self.vertices[0],
		len(self.vertices) * size_of(shape_vertex2di64),
	)
	intrinsics.mem_copy_non_overlapping(
		&res.indices[0],
		&self.indices[0],
		len(self.indices) * size_of(u32),
	)
	return
}

raw_shape_clone :: proc(
	self: ^raw_shape,
	allocator := context.allocator,
) -> (
	res: ^raw_shape = nil,
	err: runtime.Allocator_Error,
) #optional_allocator_error {
	res = new(raw_shape, allocator) or_return
	defer if err != nil {
		free(res, allocator)
		res = nil
	}

	res.vertices = utils_private.make_non_zeroed_slice(
		[]shape_vertex2d,
		len(self.vertices),
		allocator,
	) or_return
	defer if err != nil do delete(res.vertices, allocator)

	res.indices = utils_private.make_non_zeroed_slice(
		[]u32,
		len(self.indices),
		allocator,
	) or_return

	intrinsics.mem_copy_non_overlapping(
		&res.vertices[0],
		&self.vertices[0],
		len(self.vertices) * size_of(shape_vertex2d),
	)
	intrinsics.mem_copy_non_overlapping(
		&res.indices[0],
		&self.indices[0],
		len(self.indices) * size_of(u32),
	)
	return
}

GetCubicCurveType :: proc "contextless" (
	start: [2]$T,
	control0: [2]T,
	control1: [2]T,
	end: [2]T,
) -> (
	type: curve_type = .Unknown,
	d0: T,
	d1: T,
	d2: T,
	err: shape_error = nil,
) where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	if start == control0 && control0 == control1 && control1 == end {
		err = .IsPointNotLine
		return
	}

	when intrinsics.type_is_specialization_of(T, fixed.Fixed) {
		using fixed
		cross_1 := [3]T {
			sub(end.y, control1.y),
			sub(control1.x, end.x),
			sub(mul(end.x, control1.y), mul(end.y, control1.x)),
		}
		cross_2 := [3]T {
			sub(start.y, end.y),
			sub(end.x, start.x),
			sub(mul(start.x, end.y), mul(start.y, end.x)),
		}
		cross_3 := [3]T {
			sub(control0.y, start.y),
			sub(start.x, control0.x),
			sub(mul(control0.x, start.y), mul(control0.y, start.x)),
		}

		a1 := add(add(mul(start.x, cross_1.x), mul(start.y, cross_1.y)), cross_1.z)
		a2 := add(add(mul(control0.x, cross_2.x), mul(control0.y, cross_2.y)), cross_2.z)
		a3 := add(add(mul(control1.x, cross_3.x), mul(control1.y, cross_3.y)), cross_3.z)

		d0 = T {
			i = a1.i - 2 * a2.i + 3 * a3.i,
		}
		d1 = T {
			i = -a2.i + 3 * a3.i,
		}
		d2 = T {
			i = 3 * a3.i,
		}

		D := T {
			i = 3 * mul(d1, d1).i - 4 * mul(d2, d0).i,
		}
		discr := mul(mul(d0, d0), D)

		if discr.i == 0 {
			if d0.i == 0 && d1.i == 0 {
				if d2.i == 0 {
					type = .Line
					return
				}
				type = .Quadratic
				return
			}
			type = .Cusp
			return
		}
		if discr.i > 0 {
			type = .Serpentine
			return
		}
		type = .Loop
		return
	} else { 	// float
		cross_1 := [3]T {
			end.y - control1.y,
			control1.x - end.x,
			end.x * control1.y - end.y * control1.x,
		}
		cross_2 := [3]T{start.y - end.y, end.x - start.x, start.x * end.y - start.y * end.x}
		cross_3 := [3]T {
			control0.y - start.y,
			start.x - control0.x,
			control0.x * start.y - control0.y * start.x,
		}

		a1 := start.x * cross_1.x + start.y * cross_1.y + cross_1.z //9
		a2 := control0.x * cross_2.x + control0.y * cross_2.y + cross_2.z //7
		a3 := control1.x * cross_3.x + control1.y * cross_3.y + cross_3.z //7

		d0 = a1 - 2.0 * a2 + 3.0 * a3 //27
		d1 = -a2 + 3.0 * a3 //16
		d2 = 3.0 * a3 //8

		D := 3.0 * d1 * d1 - 4.0 * d2 * d0 //33 + 36 + 1 = 70
		discr := d0 * d0 * D //27 + 27 + 70 = 124

		EP :: epsilon(T) * 250.0 // about count float operations
		if discr >= -EP && discr <= EP {
			if d0 >= -EP && d0 <= EP && d1 >= -EP && d1 <= EP {
				if d2 >= -EP && d2 <= EP {
					type = .Line
					return
				}
				type = .Quadratic
				return
			}
			type = .Cusp
			return
		}
		if discr > EP {
			type = .Serpentine
			return
		}
		type = .Loop
		return
	}
	return
}

@(private = "file")
_Shapes_ComputeLine :: proc(
	vertList: ^[dynamic]shape_vertex2di64,
	indList: ^[dynamic]u32,
	color: linalg.Vector4f32,
	pts: [][2]FixedDef,
	type: curve_type,
) -> shape_error {

	using fixed

	curveType := type
	err: shape_error = nil
	//loop_reverse := false
	//if curveType == .LoopReverse do loop_reverse = true

	reverse := false
	d0, d1, d2: FixedDef
	if curveType != .Line && curveType != .Quadratic {
		curveType, d0, d1, d2 = GetCubicCurveType(pts[0], pts[1], pts[2], pts[3]) or_return
	} else if curveType == .Quadratic && len(pts) == 3 {
		vlen: u32 = u32(len(vertList))
		if GetPolygonOrientation(pts) == .CounterClockwise {
			non_zero_append(
				vertList,
				shape_vertex2di64 {
					uvw   = {0.0, 0.0, -100.0}, //-100 check this is not cubic curve
					pos   = pts[0],
					color = color,
				},
			)
			non_zero_append(
				vertList,
				shape_vertex2di64{uvw = {-0.5, 0.0, -100.0}, pos = pts[1], color = color},
			)
			non_zero_append(
				vertList,
				shape_vertex2di64{uvw = {-1.0, -1.0, -100.0}, pos = pts[2], color = color},
			)
		} else {
			non_zero_append(
				vertList,
				shape_vertex2di64{uvw = {0.0, 0.0, -100.0}, pos = pts[0], color = color},
			)
			non_zero_append(
				vertList,
				shape_vertex2di64{uvw = {0.5, 0.0, -100.0}, pos = pts[1], color = color},
			)
			non_zero_append(
				vertList,
				shape_vertex2di64{uvw = {1.0, 1.0, -100.0}, pos = pts[2], color = color},
			)
		}
		non_zero_append(indList, vlen, vlen + 1, vlen + 2)
		return nil
	}
	F: [4][3]f32

	reverseOrientation :: #force_inline proc "contextless" (F: [4][3]f32) -> [4][3]f32 {
		return [4][3]f32 {
			{-F[0][0], -F[0][1], F[0][2]},
			{-F[1][0], -F[1][1], F[1][2]},
			{-F[2][0], -F[2][1], F[2][2]},
			{-F[3][0], -F[3][1], F[3][2]},
		}
	}

	#partial switch curveType {
	case .Line:
		return nil
	case .Quadratic:
		F = {
			{{}, {}, {}},
			{1.0 / 3.0, {}, 1.0 / 3.0},
			{2.0 / 3.0, 1.0 / 3.0, 2.0 / 3.0},
			{1.0, 1.0, 1.0},
		}
		if d2.i < 0 do reverse = true
	case .Serpentine:
		d0f: f32 = f32(fixed.to_f64(d0))
		d1f: f32 = f32(fixed.to_f64(d1))
		d2f: f32 = f32(fixed.to_f64(d2))
		t1 := math.sqrt(9 * d1f * d1f - 12 * d0f * d2f)
		ls := 3 * d1f - t1
		lt := 6 * d0f

		ms := 3 * d1f + t1
		mt := lt
		ltMinusLs := lt - ls
		mtMinusMs := mt - ms

		F = {
			{ls * ms, ls * ls * ls, ms * ms * ms},
			{
				(1.0 / 3.0) * (3.0 * ls * ms - ls * mt - lt * ms),
				ls * ls * (ls - lt),
				ms * ms * (ms - mt),
			},
			{
				(1.0 / 3.0) * (lt * (mt - 2.0 * ms) + ls * (3.0 * ms - 2.0 * mt)),
				ltMinusLs * ltMinusLs * ls,
				mtMinusMs * mtMinusMs * ms,
			},
			{
				ltMinusLs * mtMinusMs,
				-(ltMinusLs * ltMinusLs * ltMinusLs),
				-(mtMinusMs * mtMinusMs * mtMinusMs),
			},
		}

		if d0.i < 0 do reverse = true
	case .Loop:
		d0f: f32 = f32(fixed.to_f64(d0))
		d1f: f32 = f32(fixed.to_f64(d1))
		d2f: f32 = f32(fixed.to_f64(d2))

		t1 := math.sqrt_f32(4 * d0f * d2f - 3 * d1f * d1f)
		ls := d1f - t1
		lt := 2 * d0f
		ms := d1f + t1
		mt := lt

		ql := ls / lt
		qm := ms / mt

		ltMinusLs := lt - ls
		mtMinusMs := mt - ms

		F = {
			{ls * ms, ls * ls * ms, ls * ms * ms},
			{
				(1.0 / 3.0) * (-ls * mt - lt * ms + 3.0 * ls * ms),
				-(1.0 / 3.0) * ls * (ls * (mt - 3.0 * ms) + 2.0 * lt * ms),
				-(1.0 / 3.0) * ms * (ls * (2.0 * mt - 3.0 * ms) + lt * ms),
			},
			{
				(1.0 / 3.0) * (lt * (mt - 2.0 * ms) + ls * (3.0 * ms - 2.0 * mt)),
				(1.0 / 3.0) * ltMinusLs * (ls * (2.0 * mt - 3.0 * ms) + lt * ms),
				(1.0 / 3.0) * mtMinusMs * (ls * (mt - 3.0 * ms) + 2.0 * lt * ms),
			},
			{
				ltMinusLs * mtMinusMs,
				-(ltMinusLs * ltMinusLs) * mtMinusMs,
				-ltMinusLs * mtMinusMs * mtMinusMs,
			},
		}

		reverse = (d0.i > 0 && F[1][0] < 0.0) || (d0.i < 0 && F[1][0] > 0.0) //? F[1][0]이 오차가생길수 있지만 일단 이렇게
	case .Cusp:
		d1f: f32 = f32(fixed.to_f64(d1))
		d2f: f32 = f32(fixed.to_f64(d2))
		ls := d2f
		lt := 3.0 * d1f
		lsMinusLt := ls - lt
		F = {
			{ls, ls * ls * ls, 1.0},
			{(ls - (1.0 / 3.0) * lt), ls * ls * lsMinusLt, 1.0},
			{ls - (2.0 / 3.0) * lt, lsMinusLt * lsMinusLt * ls, 1.0},
			{lsMinusLt, lsMinusLt * lsMinusLt * lsMinusLt, 1.0},
		}
	//reverse = true
	// case .Unknown:
	//     unreachable()
	}

	//if loop_reverse do reverse = !reverse
	if reverse {
		F = reverseOrientation(F)
	}

	appendLine :: proc(
		vertList: ^[dynamic]shape_vertex2di64,
		indList: ^[dynamic]u32,
		color: linalg.Vector4f32,
		pts: [][2]FixedDef,
		F: [4][3]f32,
	) {
		if len(pts) == 2 {
			return
		}
		start: u32 = u32(len(vertList))
		non_zero_append(
			vertList,
			shape_vertex2di64{uvw = {F[0][0], F[0][1], F[0][2]}, color = color},
		)
		non_zero_append(
			vertList,
			shape_vertex2di64{uvw = {F[1][0], F[1][1], F[1][2]}, color = color},
		)
		non_zero_append(
			vertList,
			shape_vertex2di64{uvw = {F[2][0], F[2][1], F[2][2]}, color = color},
		)
		non_zero_append(
			vertList,
			shape_vertex2di64{uvw = {F[3][0], F[3][1], F[3][2]}, color = color},
		)
		vertList[start].pos = pts[0]
		vertList[start + 1].pos = pts[1]
		vertList[start + 2].pos = pts[2]
		vertList[start + 3].pos = pts[3]
		//triangulate
		for i: u32 = 0; i < 4; i += 1 {
			for j: u32 = i + 1; j < 4; j += 1 {
				if vertList[start + i].pos == vertList[start + j].pos {
					indices: [3]u32 = {start, start, start}
					idx: u32 = 0
					for k: u32 = 0; k < 4; k += 1 {
						if k != j {
							indices[idx] += k
							idx += 1
						}
					}
					non_zero_append(indList, ..indices[:])
					return
				}
			}
		}
		for i: u32 = 0; i < 4; i += 1 {
			indices: [3]u32 = {start, start, start}
			idx: u32 = 0
			for j: u32 = 0; j < 4; j += 1 {
				if j != i {
					indices[idx] += j
					idx += 1
				}
			}
			if PointInTriangle(
				vertList[start + i].pos,
				vertList[indices[0]].pos,
				vertList[indices[1]].pos,
				vertList[indices[2]].pos,
			) {
				for k: u32 = 0; k < 3; k += 1 {
					non_zero_append(indList, indices[k])
					non_zero_append(indList, indices[(k + 1) % 3])
					non_zero_append(indList, start + i)
				}
				return
			}
		}

		b := LinesIntersect3(
			vertList[start].pos,
			vertList[start + 2].pos,
			vertList[start + 1].pos,
			vertList[start + 3].pos,
		)
		if b == .intersect {
			if fixed_ex.length2(fixed_ex.sub2(vertList[start + 2].pos, vertList[start].pos)).i <
			   fixed_ex.length2(fixed_ex.sub2(vertList[start + 3].pos, vertList[start + 1].pos)).i {
				non_zero_append(indList, start, start + 1, start + 2, start, start + 2, start + 3)
			} else {
				non_zero_append(
					indList,
					start,
					start + 1,
					start + 3,
					start + 1,
					start + 2,
					start + 3,
				)
			}
			return
		}
		b = LinesIntersect3(
			vertList[start].pos,
			vertList[start + 3].pos,
			vertList[start + 1].pos,
			vertList[start + 2].pos,
		)
		if b == .intersect {
			if fixed_ex.length2(fixed_ex.sub2(vertList[start + 3].pos, vertList[start].pos)).i <
			   fixed_ex.length2(fixed_ex.sub2(vertList[start + 2].pos, vertList[start + 1].pos)).i {
				non_zero_append(indList, start, start + 1, start + 3, start, start + 3, start + 2)
			} else {
				non_zero_append(
					indList,
					start,
					start + 1,
					start + 2,
					start + 2,
					start + 1,
					start + 3,
				)
			}
			return
		}
		if fixed_ex.length2(fixed_ex.sub2(vertList[start + 1].pos, vertList[start].pos)).i <
		   fixed_ex.length2(fixed_ex.sub2(vertList[start + 3].pos, vertList[start + 2].pos)).i {
			non_zero_append(indList, start, start + 2, start + 1, start, start + 1, start + 3)
		} else {
			non_zero_append(indList, start, start + 2, start + 3, start + 3, start + 2, start + 1)
		}
	}
	appendLine(vertList, indList, color, pts[:len(pts)], F)

	return nil
}

@(require_results)
cvt_raw_shapei64_to_raw_shape :: proc(
	raw64: raw_shapei64,
	allocator := context.allocator,
) -> (
	res: raw_shape,
	err: runtime.Allocator_Error,
) #optional_allocator_error {
	res.vertices = utils_private.make_non_zeroed_slice(
		[]shape_vertex2d,
		len(raw64.vertices),
		allocator,
	) or_return
	defer if err != nil do delete(res.vertices, allocator)
	res.indices = utils_private.make_non_zeroed_slice(
		[]u32,
		len(raw64.indices),
		allocator,
	) or_return
	defer if err != nil do delete(res.indices, allocator)

	for v, i in raw64.vertices {
		res.vertices[i].pos.x = f32(fixed.to_f64(v.pos.x))
		res.vertices[i].pos.y = f32(fixed.to_f64(v.pos.y))

		res.vertices[i].color = v.color
		res.vertices[i].uvw = v.uvw
	}
	mem.copy_non_overlapping(
		raw_data(res.indices),
		raw_data(raw64.indices),
		len(raw64.indices) * size_of(u32),
	)

	return
}

@(require_results)
cvt_raw_shape_to_raw_shapei64 :: proc(
	raw: raw_shape,
	allocator := context.allocator,
) -> (
	res: raw_shapei64,
	err: runtime.Allocator_Error,
) #optional_allocator_error {
	res.vertices = utils_private.make_non_zeroed_slice(
		[]shape_vertex2di64,
		len(raw.vertices),
		allocator,
	) or_return
	defer if err != nil do delete(res.vertices, allocator)
	res.indices = utils_private.make_non_zeroed_slice([]u32, len(raw.indices), allocator) or_return
	defer if err != nil do delete(res.indices, allocator)

	for v, i in raw.vertices {
		fixed.init_from_f64(&res.vertices[i].pos.x, f64(v.pos.x))
		fixed.init_from_f64(&res.vertices[i].pos.y, f64(v.pos.y))

		res.vertices[i].color = v.color
		res.vertices[i].uvw = v.uvw
	}
	mem.copy_non_overlapping(
		raw_data(res.indices),
		raw_data(raw.indices),
		len(res.indices) * size_of(u32),
	)

	return
}

@(require_results)
cvt_shapes_to_shapesi64 :: proc(
	poly: shapes,
	allocator := context.allocator,
) -> (
	poly64: shapesi64,
	err: runtime.Allocator_Error,
) #optional_allocator_error {
	poly64 = shapesi64 {
		nodes = utils_private.make_non_zeroed_slice(
			[]shape_nodei64,
			len(poly.nodes),
			allocator,
		) or_return,
	}
	defer if err != nil {
		delete(poly64.nodes, allocator)
	}

	for n, i in poly.nodes {
		defer if err != nil { 	//(1)
			for j in 0 ..< i {
				for p in poly64.nodes[j].pts do delete(p, allocator)
				delete(poly64.nodes[j].pts, allocator)
				if poly64.nodes[j].curve_pts_ids != nil {
					for p in poly64.nodes[j].curve_pts_ids do delete(p, allocator)
					delete(poly64.nodes[j].curve_pts_ids, allocator)
				}
			}
		}
		poly64.nodes[i].pts = utils_private.make_non_zeroed_slice(
			[][][2]FixedDef,
			len(n.pts),
			allocator,
		) or_return
		defer if err != nil {
			delete(poly64.nodes[i].pts, allocator)
		}

		for p, j in n.pts {
			defer if err != nil {
				for e in 0 ..< j {
					delete(poly64.nodes[i].pts[e], allocator)
				}
			}
			poly64.nodes[i].pts[j] = utils_private.make_non_zeroed_slice(
				[][2]FixedDef,
				len(p),
				allocator,
			) or_return

			for pp, e in p {
				fixed.init_from_f64(&poly64.nodes[i].pts[j][e].x, f64(pp.x))
				fixed.init_from_f64(&poly64.nodes[i].pts[j][e].y, f64(pp.y))
			}
		}

		defer if err != nil { 	//(1)에서 현재 i pts는 지우지 않는다.
			for p in poly64.nodes[i].pts do delete(p, allocator)
		}

		if n.curve_pts_ids != nil {
			poly64.nodes[i].curve_pts_ids = utils_private.make_non_zeroed_slice(
				[][]u32,
				len(n.curve_pts_ids),
				allocator,
			) or_return
			defer if err != nil {
				delete(poly64.nodes[i].curve_pts_ids, allocator)
			}

			for p, j in n.curve_pts_ids {
				defer if err != nil {
					for e in 0 ..< j {
						delete(poly64.nodes[i].curve_pts_ids[e], allocator)
					}
				}
				poly64.nodes[i].curve_pts_ids[j] = utils_private.make_non_zeroed_slice(
					[]u32,
					len(p),
					allocator,
				) or_return
				mem.copy_non_overlapping(
					raw_data(poly64.nodes[i].curve_pts_ids[j]),
					raw_data(p),
					len(p) * size_of(u32),
				)
			}
		} else {
			poly64.nodes[i].curve_pts_ids = nil
		}

		poly64.nodes[i].is_closed = n.is_closed
		fixed.init_from_f64(&poly64.nodes[i].thickness, n.thickness)
		poly64.nodes[i].color = n.color
		poly64.nodes[i].stroke_color = n.stroke_color
	}
	return
}

@(require_results)
cvt_shapesi64_to_shapes :: proc(
	poly: shapesi64,
	allocator := context.allocator,
) -> (
	poly32: shapes,
	err: runtime.Allocator_Error,
) #optional_allocator_error {
	poly32 = shapes {
		nodes = utils_private.make_non_zeroed_slice(
			[]shape_node,
			len(poly.nodes),
			allocator,
		) or_return,
	}
	defer if err != nil {
		delete(poly32.nodes, allocator)
	}

	for n, i in poly.nodes {
		defer if err != nil {
			for j in 0 ..< i {
				for p in poly32.nodes[j].pts do delete(p, allocator)
				delete(poly32.nodes[j].pts, allocator)
				if poly32.nodes[j].curve_pts_ids != nil {
					for p in poly32.nodes[j].curve_pts_ids do delete(p, allocator)
					delete(poly32.nodes[j].curve_pts_ids, allocator)
				}
			}
		}
		poly32.nodes[i].pts = utils_private.make_non_zeroed_slice(
			[][]linalg.Vector2f32,
			len(n.pts),
			allocator,
		) or_return
		defer if err != nil {
			delete(poly32.nodes[i].pts, allocator)
		}

		for p, j in n.pts {
			defer if err != nil {
				for e in 0 ..< j {
					delete(poly32.nodes[i].pts[e], allocator)
				}
			}
			poly32.nodes[i].pts[j] = utils_private.make_non_zeroed_slice(
				[]linalg.Vector2f32,
				len(p),
				allocator,
			) or_return

			for pp, e in p {
				poly32.nodes[i].pts[j][e].x = f32(fixed.to_f64(pp.x))
				poly32.nodes[i].pts[j][e].y = f32(fixed.to_f64(pp.y))
			}
		}

		defer if err != nil {
			for p in poly32.nodes[i].pts do delete(p, allocator)
		}

		if n.curve_pts_ids != nil {
			poly32.nodes[i].curve_pts_ids = utils_private.make_non_zeroed_slice(
				[][]u32,
				len(n.curve_pts_ids),
				allocator,
			) or_return
			defer if err != nil {
				delete(poly32.nodes[i].curve_pts_ids, allocator)
			}

			for p, j in n.curve_pts_ids {
				defer if err != nil {
					for e in 0 ..< j {
						delete(poly32.nodes[i].curve_pts_ids[e], allocator)
					}
				}
				poly32.nodes[i].curve_pts_ids[j] = utils_private.make_non_zeroed_slice(
					[]u32,
					len(p),
					allocator,
				) or_return
				mem.copy_non_overlapping(
					raw_data(poly32.nodes[i].curve_pts_ids[j]),
					raw_data(p),
					len(p) * size_of(u32),
				)
			}
		} else {
			poly32.nodes[i].curve_pts_ids = nil
		}

		poly32.nodes[i].is_closed = n.is_closed
		poly32.nodes[i].thickness = fixed.to_f64(n.thickness)
		poly32.nodes[i].color = n.color
		poly32.nodes[i].stroke_color = n.stroke_color
	}
	return
}

shapes_compute_polygon :: proc(
	poly: shapes,
	allocator := context.allocator,
) -> (
	res: raw_shape,
	err: shape_error = nil,
) {
	poly64 := cvt_shapes_to_shapesi64(poly, allocator) or_return

	res64 := shapes_compute_polygoni64(poly64, context.temp_allocator) or_return

	res.vertices = utils_private.make_non_zeroed_slice(
		[]shape_vertex2d,
		len(res64.vertices),
		allocator,
	) or_return
	defer if err != nil do delete(res.vertices, allocator)
	res.indices = utils_private.make_non_zeroed_slice(
		[]u32,
		len(res64.indices),
		allocator,
	) or_return
	defer if err != nil do delete(res.indices, allocator) // not working

	for v, i in res64.vertices {
		res.vertices[i].pos.x = f32(fixed.to_f64(v.pos.x))
		res.vertices[i].pos.y = f32(fixed.to_f64(v.pos.y))

		res.vertices[i].color = v.color
		res.vertices[i].uvw = v.uvw
	}

	mem.copy_non_overlapping(
		raw_data(res.indices),
		raw_data(res64.indices),
		len(res64.indices) * size_of(u32),
	)
	return
}

@(private)
CURVE_STRUCT :: struct {
	start: [2]FixedDef,
	ctl0:  [2]FixedDef,
	ctl1:  [2]FixedDef,
	end:   [2]FixedDef,
	type:  curve_type,
}

shapes_compute_polygoni64 :: proc(
	poly: shapesi64,
	allocator := context.allocator,
) -> (
	res: raw_shapei64,
	err: shape_error = nil,
) {
	vertList: [dynamic]shape_vertex2di64 = make([dynamic]shape_vertex2di64, context.temp_allocator)
	indList: [dynamic]u32 = make([dynamic]u32, context.temp_allocator)

	shapes_compute_polygon_in :: proc(
		vertList: ^[dynamic]shape_vertex2di64,
		indList: ^[dynamic]u32,
		poly: shapesi64,
		allocator: runtime.Allocator,
	) -> (
		err: shape_error = nil,
	) {
		using fixed

		non_curves := make([dynamic][][2]FixedDef, context.temp_allocator)
		non_curves2: [dynamic][dynamic][2]FixedDef = make(
			[dynamic][dynamic][2]FixedDef,
			context.temp_allocator,
		)
		non_curves_ccw2: [dynamic]PolyOrientation = make(
			[dynamic]PolyOrientation,
			context.temp_allocator,
		)
		curves2: [dynamic][dynamic]CURVE_STRUCT = make(
			[dynamic][dynamic]CURVE_STRUCT,
			context.temp_allocator,
		)
		insert_ar := make([dynamic][2]FixedDef, context.temp_allocator)

		for node, nidx in poly.nodes {
			if node.color.a > 0 { 	//TODO Handle If is_close == false
				non_zero_resize_dynamic_array(&non_curves2, len(node.pts)) or_return
				non_zero_resize_dynamic_array(&non_curves_ccw2, 0) or_return
				non_zero_resize_dynamic_array(&curves2, len(node.pts)) or_return

				for i in 0 ..< len(node.pts) {
					if non_curves2[i] == nil {
						non_curves2[i] = make(
							[dynamic][2]FixedDef,
							context.temp_allocator,
						) or_return
						curves2[i] = make([dynamic]CURVE_STRUCT, context.temp_allocator) or_return
					} else {
						non_zero_resize_dynamic_array(&non_curves2[i], 0) or_return
						non_zero_resize_dynamic_array(&curves2[i], 0) or_return
					}
				}
				for np, npi in node.pts {
					curve_idx := 0
					for pt, i in np {
						if node.curve_pts_ids != nil &&
						   curve_idx < len(node.curve_pts_ids[npi]) &&
						   node.curve_pts_ids[npi][curve_idx] == u32(i) {
							if curve_idx + 1 < len(node.curve_pts_ids[npi]) &&
							   node.curve_pts_ids[npi][curve_idx + 1] == u32(i) {
								non_zero_append(
									&curves2[npi],
									CURVE_STRUCT {
										start = np[i - 1],
										ctl0 = np[i],
										ctl1 = np[i + 1],
										end = np[(i + 2) % len(np)],
										type = .Unknown,
									},
								) or_return
								curve_idx += 2
							} else {
								non_zero_append(
									&curves2[npi],
									CURVE_STRUCT {
										start = np[i - 1],
										ctl0 = np[i],
										end = np[(i + 1) % len(np)],
										type = .Quadratic,
									},
								) or_return
								curve_idx += 1
							}
						} else {
							non_zero_append(&non_curves2[npi], pt) or_return
						}
					}
				}
				// Loop subdivision: subdivide cubic Loop curves per polygon
				for npi in 0 ..< len(curves2) {
					curves_npi := &curves2[npi]
					for i := 0; i < len(curves_npi); i += 1 {
						c := curves_npi[i]
						if c.type != .Quadratic {
							curveType, d0, d1, d2 := GetCubicCurveType(
								c.start,
								c.ctl0,
								c.ctl1,
								c.end,
							) or_return
							if curveType == .Loop {
								t1 := FixedDef {
									i = utils_private.sqrt_i64(
										(4 * d0.i * d2.i - 3 * d1.i * d1.i),
										//<< FIXED_SHIFT >> FIXED_SHIFT, //결국 똑같음
									),
								}
								ls := sub(d1, t1)
								lt := mul(Fixed2, d0)
								ms := add(d1, t1)
								mt := lt
								ql := ls //div(ls, lt) // loop면 d0이 0이 아니라서 0으로 나누지 않는다.
								qm := ms //div(ms, mt)
								subdiv_at: FixedDef
								if lt.i > 0 ? (0 < ql.i && ql.i < lt.i) : (0 > ql.i && ql.i > lt.i) {
									subdiv_at = div(ls, lt)
								} else if mt.i > 0 ? (0 < qm.i && qm.i < mt.i) : (0 > qm.i && qm.i > mt.i) {
									subdiv_at = div(ms, mt)
								} else {
									continue
								}
								pt01, pt12, pt23, pt012, pt123, pt0123 := SubdivCubicBezier(
									[4][2]FixedDef{c.start, c.ctl0, c.ctl1, c.end},
									subdiv_at,
								)
								curves_npi[i] = CURVE_STRUCT {
									start = c.start,
									ctl0  = pt01,
									ctl1  = pt012,
									end   = pt0123,
									type  = .Unknown,
								}
								utils_private.non_zero_inject_at_elem(
									curves_npi,
									i + 1,
									CURVE_STRUCT {
										start = pt0123,
										ctl0 = pt123,
										ctl1 = pt23,
										end = c.end,
										type = .Unknown,
									},
								) or_return
								i += 1 //add extra 1
							}
						}
					}
				}
				// Overlap check: subdivide overlapping curves across all polygons
				for {
					has_overlap := false
					for npi in 0 ..< len(curves2) {
						curves_i := &curves2[npi]
						for i := 0; i < len(curves_i); i += 1 {
							for npj in 0 ..< len(curves2) {
								curves_j := &curves2[npj]
								for j := 0; j < len(curves_j); j += 1 {
									if npi == npj && i == j do continue
									cur := curves_i[i]
									cur2 := curves_j[j]
									poly1 :=
										cur.type == .Quadratic ? [][2]FixedDef{cur.start, cur.ctl0, cur.end} : [][2]FixedDef{cur.start, cur.ctl0, cur.ctl1, cur.end}
									poly2 :=
										cur2.type == .Quadratic ? [][2]FixedDef{cur2.start, cur2.ctl0, cur2.end} : [][2]FixedDef{cur2.start, cur2.ctl0, cur2.ctl1, cur2.end}
									if !PolygonOverlapsPolygon(poly1, poly2) do continue
									subdiv_t: FixedDef
									subdiv_t_: FixedDef
									if cur2.type == .Quadratic {
										_, subdiv_t, subdiv_t_ = PointInLine(
											cur2.ctl0,
											cur.start,
											cur.end,
										)
									} else {
										_, subdiv_t, subdiv_t_ = PointInLine(
											lerp_fixed(
												cur2.ctl0,
												cur2.ctl1,
												splat_2_fixed(Div2Fixed),
											),
											cur.start,
											cur.end,
										)
									}
									if cur.type == .Quadratic {
										pt01, pt12, pt012 := SubdivQuadraticBezier(
											[3][2]FixedDef{cur.start, cur.ctl0, cur.end},
											fixed.div(subdiv_t, subdiv_t_),
										)
										curves_i[i] = CURVE_STRUCT {
											start = cur.start,
											ctl0  = pt01,
											end   = pt012,
											type  = .Quadratic,
										}
										utils_private.non_zero_inject_at_elem(
											curves_i,
											i + 1,
											CURVE_STRUCT {
												start = pt012,
												ctl0 = pt12,
												end = cur.end,
												type = .Quadratic,
											},
										) or_return
									} else {
										pt01, pt12, pt23, pt012, pt123, pt0123 :=
											SubdivCubicBezier(
												[4][2]FixedDef {
													cur.start,
													cur.ctl0,
													cur.ctl1,
													cur.end,
												},
												fixed.div(subdiv_t, subdiv_t_),
											)
										curves_i[i] = CURVE_STRUCT {
												start = cur.start,
												ctl0  = pt01,
												ctl1  = pt012,
												end   = pt0123,
												type  = .Unknown,
											}
										utils_private.non_zero_inject_at_elem(
											curves_i,
											i + 1,
											CURVE_STRUCT {
												start = pt0123,
												ctl0 = pt123,
												ctl1 = pt23,
												end = cur.end,
												type = .Unknown,
											},
										) or_return
									}
									if i < j do j += 1
									i += 1
									has_overlap = true
								}
							}
						}
					}
					if !has_overlap do break
				}

				// Insert curve control points into polygon boundaries (per-polygon)
				for npi in 0 ..< len(node.pts) {
					non_curves_npi := &non_curves2[npi]
					curves_npi := curves2[npi]
					curve_idx := 0
					poly_pts := non_curves_npi[:]
					orient := GetPolygonOrientation(poly_pts)
					invent_b := orient != .CounterClockwise

					i := 0
					for i < len(non_curves_npi) {
						non := non_curves_npi[i]
						next: int
						if node.is_closed {
							next = (i + 1) % len(non_curves_npi)
						} else {
							if i + 1 >= len(non_curves_npi) do break
							next = i + 1
						}
						non_next := non_curves_npi[next]

						if curve_idx < len(curves_npi) && non == curves_npi[curve_idx].start {
							j := curve_idx + 1
							for ; j < len(curves_npi) && non_next != curves_npi[j].start; j += 1 {}
							non_zero_resize_dynamic_array(&insert_ar, 0) or_return

							if utils_private.invent_bool(
								PointInPolygon(curves_npi[curve_idx].ctl0, poly_pts),
								invent_b,
							) {
								non_zero_append(&insert_ar, curves_npi[curve_idx].ctl0) or_return
							}
							if curves_npi[curve_idx].type != .Quadratic {
								if utils_private.invent_bool(
									PointInPolygon(curves_npi[curve_idx].ctl1, poly_pts),
									invent_b,
								) {
									non_zero_append(
										&insert_ar,
										curves_npi[curve_idx].ctl1,
									) or_return
								}
							}
							for e := curve_idx + 1; e < j; e += 1 {
								non_zero_append(&insert_ar, curves_npi[e].start) or_return
								if utils_private.invent_bool(
									PointInPolygon(curves_npi[e].ctl0, poly_pts),
									invent_b,
								) {
									non_zero_append(&insert_ar, curves_npi[e].ctl0) or_return
								}
								if curves_npi[e].type != .Quadratic {
									if utils_private.invent_bool(
										PointInPolygon(curves_npi[e].ctl1, poly_pts),
										invent_b,
									) {
										non_zero_append(&insert_ar, curves_npi[e].ctl1) or_return
									}
								}
							}

							utils_private.non_zero_inject_at_elems(
								non_curves_npi,
								next,
								..insert_ar[:],
							) or_return
							poly_pts = non_curves_npi[:]

							if j >= len(curves_npi) do break
							curve_idx = j

							i = next + len(insert_ar)
						} else {
							i += 1
						}
					}
					non_zero_append(
						&non_curves_ccw2,
						GetPolygonOrientation(non_curves2[npi][:]),
					) or_return
				}
				non_zero_resize_dynamic_array(&non_curves, len(non_curves2)) or_return
				for npi in 0 ..< len(non_curves2) {
					non_curves[npi] = non_curves2[npi][:]
				}
				indices, tri_err := TrianguatePolygons_Fixed(
					non_curves[:],
					non_curves_ccw2[:],
					allocator,
				)
				if tri_err != nil {
					switch tri in tri_err {
					case __Trianguate_Error:
						err = tri_err.(__Trianguate_Error)
					case runtime.Allocator_Error:
						err = tri_err.(runtime.Allocator_Error)
					case __Geometry_Error:
						err = tri_err.(__Geometry_Error)
					}
					return
				}

				//

				//TODO curves _Shapes_ComputeLine
			}
		}
		return
	}
	return
}

