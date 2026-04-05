package geometry

import "./clipper"
import "./linalg_ex"
import "./triangulation"
import "base:intrinsics"
import "base:runtime"
import "core:fmt"
import "core:math"
import "core:math/linalg"
import "core:mem"

import "engine:utils_private/fixed_bcd"

import "engine:utils_private"


shape_vertex_flag :: enum u8 {
	CUBIC,
	QUAD,
	LINE,
}

shape_vertex2d :: struct {
	pos:   linalg.Vector2f32,
	uvw:   linalg.Vector3f32,
	color: linalg.Vector4f32,
	flag:  shape_vertex_flag,
}

raw_shape :: struct {
	vertices: []shape_vertex2d,
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

shape_error :: union #shared_nil {
	__shape_error,
	triangulation.__Trianguate_Error,
	clipper.__Clipper_Error,
	runtime.Allocator_Error,
}

shape_node :: struct {
	pts:          [][]linalg.Vector2f32,
	is_curves:    [][]bool,
	color:        linalg.Vector4f32,
	stroke_color: linalg.Vector4f32,
	thickness:    f32,
	is_closed:    bool,
}

shapes :: struct {
	nodes: []shape_node,
}

// Same layout as curve_struct_fixed for floating-point coordinates (f16 / f32 / f64).
@(private)
curve_struct_float :: struct($F: typeid) where intrinsics.type_is_float(F) {
	start: [2]F,
	ctl0:  [2]F,
	ctl1:  [2]F,
	end:   [2]F,
	type:  curve_type,
}

raw_shape_free :: proc(self: raw_shape, allocator := context.allocator) {
	delete(self.vertices, allocator)
	delete(self.indices, allocator)
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
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	if start == control0 && control0 == control1 && control1 == end {
		err = .IsPointNotLine
		return
	}

	when intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
		cross_1 := [3]T {
			fixed_bcd.sub(end.y, control1.y),
			fixed_bcd.sub(control1.x, end.x),
			fixed_bcd.sub(fixed_bcd.mul(end.x, control1.y), fixed_bcd.mul(end.y, control1.x)),
		}
		cross_2 := [3]T {
			fixed_bcd.sub(start.y, end.y),
			fixed_bcd.sub(end.x, start.x),
			fixed_bcd.sub(fixed_bcd.mul(start.x, end.y), fixed_bcd.mul(start.y, end.x)),
		}
		cross_3 := [3]T {
			fixed_bcd.sub(control0.y, start.y),
			fixed_bcd.sub(start.x, control0.x),
			fixed_bcd.sub(fixed_bcd.mul(control0.x, start.y), fixed_bcd.mul(control0.y, start.x)),
		}
		a1 := fixed_bcd.add(
			fixed_bcd.add(fixed_bcd.mul(start.x, cross_1.x), fixed_bcd.mul(start.y, cross_1.y)),
			cross_1.z,
		)
		a2 := fixed_bcd.add(
			fixed_bcd.mul(control0.x, cross_2.x),
			fixed_bcd.add(fixed_bcd.mul(control0.y, cross_2.y), cross_2.z),
		)
		a3 := fixed_bcd.add(
			fixed_bcd.mul(control1.x, cross_3.x),
			fixed_bcd.add(fixed_bcd.mul(control1.y, cross_3.y), cross_3.z),
		)
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
			i = 3 * fixed_bcd.mul(d1, d1).i - 4 * fixed_bcd.mul(d2, d0).i,
		}
		discr := fixed_bcd.mul(fixed_bcd.mul(d0, d0), D)
		if discr.i == 0 {
			if d0.i == 0 && d1.i == 0 {
				if d2.i == 0 do type = .Line
				else do type = .Quadratic
				return
			}
			type = .Cusp
			return
		}
		if discr.i > 0 do type = .Serpentine
		else do type = .Loop
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

		EP: T = linalg_ex.epsilon(T)
		if discr >= -EP && discr <= EP {
			if d0 >= -EP && d0 <= EP && d1 >= -EP && d1 <= EP {
				if d2 == 0.0 {
					type = .Line
					return
				}
				type = .Quadratic
				return
			}
			type = .Cusp
			return
		}
		if discr > 0.0 {
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
	vertList: ^[dynamic]shape_vertex2d,
	indList: ^[dynamic]u32,
	color: linalg.Vector4f32,
	pts: curve_struct_float(f32),
) -> shape_error {
	curveType := pts.type
	err: shape_error = nil
	//loop_reverse := false
	//if curveType == .LoopReverse do loop_reverse = true

	reverse := false
	d0, d1, d2: f32
	if curveType != .Line && curveType != .Quadratic {
		curveType, d0, d1, d2 = GetCubicCurveType(pts.start, pts.ctl0, pts.ctl1, pts.end) or_return
	} else if curveType == .Quadratic {
		vlen: u32 = u32(len(vertList))
		quad_sign := f32(1.0)
		if linalg_ex.GetPolygonOrientation([][2]f32{pts.start, pts.ctl0, pts.end}) ==
		   .CounterClockwise {
			quad_sign = -1.0
		}
		non_zero_append(
			vertList,
			shape_vertex2d {
				uvw = {0.0, 0.0, quad_sign},
				pos = linalg.Vector2f32{pts.start.x, pts.start.y},
				color = color,
				flag = .QUAD,
			},
		)
		non_zero_append(
			vertList,
			shape_vertex2d {
				uvw = {0.5, 0.0, quad_sign},
				pos = linalg.Vector2f32{pts.ctl0.x, pts.ctl0.y},
				color = color,
				flag = .QUAD,
			},
		)
		non_zero_append(
			vertList,
			shape_vertex2d {
				uvw = {1.0, 1.0, quad_sign},
				pos = linalg.Vector2f32{pts.end.x, pts.end.y},
				color = color,
				flag = .QUAD,
			},
		)
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
	case .Serpentine:
		t1 := math.sqrt_f32(9 * d1 * d1 - 12 * d0 * d2)
		ls := 3 * d1 - t1
		lt := 6 * d0

		ms := 3 * d1 + t1
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

		if d0 < 0 do reverse = true
	case .Loop:
		t1 := math.sqrt_f32(4 * d0 * d2 - 3 * d1 * d1)
		ls := d1 - t1
		lt := 2 * d0
		ms := d1 + t1
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

		reverse = (d0 > 0 && F[1][0] < 0.0) || (d0 < 0 && F[1][0] > 0.0)
	case .Cusp:
		ls := d2
		lt := 3.0 * d1
		lsMinusLt := ls - lt
		F = {
			{ls, ls * ls * ls, 1.0},
			{(ls - (1.0 / 3.0) * lt), ls * ls * lsMinusLt, 1.0},
			{ls - (2.0 / 3.0) * lt, lsMinusLt * lsMinusLt * ls, 1.0},
			{lsMinusLt, lsMinusLt * lsMinusLt * lsMinusLt, 1.0},
		}
	case .Quadratic:
		F = {
			{0.0, 0.0, 0.0},
			{1.0 / 3.0, 0.0, 1.0 / 3.0},
			{2.0 / 3.0, 1.0 / 3.0, 2.0 / 3.0},
			{1.0, 1.0, 1.0},
		}
		if d2 < 0 do reverse = true
	case .Line:
		return nil
	//reverse = true
	// case .Unknown:
	//     unreachable()
	}

	//if loop_reverse do reverse = !reverse
	if reverse {
		F = reverseOrientation(F)
	}

	appendLine :: proc(
		vertList: ^[dynamic]shape_vertex2d,
		indList: ^[dynamic]u32,
		color: linalg.Vector4f32,
		pts: curve_struct_float(f32),
		F: [4][3]f32,
	) -> shape_error {
		start: u32 = u32(len(vertList))
		non_zero_append(
			vertList,
			shape_vertex2d{uvw = {F[0][0], F[0][1], F[0][2]}, color = color},
		) or_return
		non_zero_append(
			vertList,
			shape_vertex2d{uvw = {F[1][0], F[1][1], F[1][2]}, color = color},
		) or_return
		non_zero_append(
			vertList,
			shape_vertex2d{uvw = {F[2][0], F[2][1], F[2][2]}, color = color},
		) or_return
		non_zero_append(
			vertList,
			shape_vertex2d{uvw = {F[3][0], F[3][1], F[3][2]}, color = color},
		) or_return

		vertList[start].pos = linalg.Vector2f32{pts.start.x, pts.start.y}
		vertList[start + 1].pos = linalg.Vector2f32{pts.ctl0.x, pts.ctl0.y}
		vertList[start + 2].pos = linalg.Vector2f32{pts.ctl1.x, pts.ctl1.y}
		vertList[start + 3].pos = linalg.Vector2f32{pts.end.x, pts.end.y}
		//triangulate
		vts: [4]linalg.Vector2f32 = {
			vertList[start].pos,
			vertList[start + 1].pos,
			vertList[start + 2].pos,
			vertList[start + 3].pos,
		}

		for i: u32 = 0; i < 4; i += 1 {
			for j: u32 = i + 1; j < 4; j += 1 {
				if vts[i] == vts[j] {
					indices: [3]u32 = {start, start, start}
					idx: u32 = 0
					for k: u32 = 0; k < 4; k += 1 {
						if k != j {
							indices[idx] += k
							idx += 1
						}
					}
					non_zero_append(indList, ..indices[:]) or_return
					return nil
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
			if linalg_ex.PointInTriangle(
				vts[i],
				vts[indices[0] - start],
				vts[indices[1] - start],
				vts[indices[2] - start],
			) {
				for k: u32 = 0; k < 3; k += 1 {
					non_zero_append(indList, indices[k]) or_return
					non_zero_append(indList, indices[(k + 1) % 3]) or_return
					non_zero_append(indList, start + i) or_return
				}
				return nil
			}
		}

		b := linalg_ex.LinesIntersect3(vts[0], vts[2], vts[1], vts[3])
		if b == .intersect {
			if linalg.vector_length2(vts[2] - vts[0]) < linalg.vector_length2(vts[3] - vts[1]) {
				non_zero_append(
					indList,
					start,
					start + 1,
					start + 2,
					start,
					start + 2,
					start + 3,
				) or_return
			} else {
				non_zero_append(
					indList,
					start,
					start + 1,
					start + 3,
					start + 1,
					start + 2,
					start + 3,
				) or_return
			}
			return nil
		}
		b = linalg_ex.LinesIntersect3(vts[0], vts[3], vts[1], vts[2])
		if b == .intersect {
			if linalg.vector_length2(vts[3] - vts[0]) < linalg.vector_length2(vts[2] - vts[1]) {
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
				) or_return
			}
			return nil
		}
		if linalg.vector_length2(vts[1] - vts[0]) < linalg.vector_length2(vts[3] - vts[2]) {
			non_zero_append(
				indList,
				start,
				start + 2,
				start + 1,
				start,
				start + 1,
				start + 3,
			) or_return
		} else {
			non_zero_append(
				indList,
				start,
				start + 2,
				start + 3,
				start + 3,
				start + 2,
				start + 1,
			) or_return
		}
		return nil
	}
	appendLine(vertList, indList, color, pts, F) or_return

	return nil
}

@(private = "file")
SubdivCurveAndInjectAt :: proc(
	curves_n: ^[dynamic]curve_struct_float(f32),
	insert_idx: int,
	cur: ^curve_struct_float(f32),
	src: curve_struct_float(f32),
	t: f32,
) -> (
	mid: [2]f32,
	c0: [2]f32,
	c1: [2]f32,
	err: shape_error = nil,
) {
	if src.type == .Quadratic {
		p0, p1_, p2_ := linalg_ex.SubdivQuadraticBezier([3][2]f32{src.start, src.ctl0, src.end}, t)
		cur.ctl0, cur.end = p0, p1_
		mid, c0 = p1_, p2_
		utils_private.non_zero_inject_at_elem(
			curves_n,
			insert_idx,
			curve_struct_float(f32){start = p1_, ctl0 = p2_, end = src.end, type = .Quadratic},
		) or_return
	} else {
		p0, p1_, p2_, p3_, p4_ := linalg_ex.SubdivCubicBezier(
			[4][2]f32{src.start, src.ctl0, src.ctl1, src.end},
			t,
		)
		cur.ctl0, cur.ctl1, cur.end = p0, p1_, p2_
		mid, c0, c1 = p2_, p3_, p4_
		utils_private.non_zero_inject_at_elem(
			curves_n,
			insert_idx,
			curve_struct_float(f32) {
				start = p2_,
				ctl0 = p3_,
				ctl1 = p4_,
				end = src.end,
				type = .Unknown,
			},
		) or_return
	}
	return
}

@(private = "file")
SubdivCurveAt :: proc "contextless" (
	src: curve_struct_float(f32),
	t: f32,
) -> (
	left, right: curve_struct_float(f32),
) {
	if src.type == .Quadratic {
		p0, p1_, p2_ := linalg_ex.SubdivQuadraticBezier([3][2]f32{src.start, src.ctl0, src.end}, t)
		left = curve_struct_float(f32) {
			start = src.start,
			ctl0  = p0,
			end   = p1_,
			type  = .Quadratic,
		}
		right = curve_struct_float(f32) {
			start = p1_,
			ctl0  = p2_,
			end   = src.end,
			type  = .Quadratic,
		}
	} else {
		p0, p1_, p2_, p3_, p4_ := linalg_ex.SubdivCubicBezier(
			[4][2]f32{src.start, src.ctl0, src.ctl1, src.end},
			t,
		)
		left = curve_struct_float(f32) {
			start = src.start,
			ctl0  = p0,
			ctl1  = p1_,
			end   = p2_,
			type  = .Unknown,
		}
		right = curve_struct_float(f32) {
			start = p2_,
			ctl0  = p3_,
			ctl1  = p4_,
			end   = src.end,
			type  = .Unknown,
		}
	}
	return
}

@(private = "file")
CurveChordOverlaps :: proc(src: curve_struct_float(f32), cur: curve_struct_float(f32)) -> bool {
	if src.type == .Line && cur.type == .Line do return false

	if src.type == .Line || cur.type == .Line {
		// curve-vs-line: use curve chord (start-end) vs line (start-end)
		k, _ := linalg_ex.LinesIntersect2(src.start, src.end, cur.start, cur.end, true)
		return k == .intersect
	}

	k1, _ := linalg_ex.LinesIntersect2(src.start, src.end, cur.start, cur.ctl0, true)
	if k1 == .intersect do return true

	k2, _ := linalg_ex.LinesIntersect2(
		src.start,
		src.end,
		cur.type == .Quadratic ? cur.ctl0 : cur.ctl1,
		cur.end,
		true,
	)
	return k2 == .intersect
}

@(private = "file")
CurveChordOverlapsAny :: proc(
	srcs: []curve_struct_float(f32),
	curs: []curve_struct_float(f32),
) -> bool {
	for src in srcs {
		for cur in curs {
			if CurveChordOverlaps(src, cur) do return true
		}
	}
	return false
}

@(private = "file")
SubdivCurveSegmentsAtHalf :: proc "contextless" (
	srcs: []curve_struct_float(f32),
	dst: []curve_struct_float(f32),
) {
	// caller guarantees len(dst) == len(srcs) * 2
	for src, idx in srcs {
		a, b := SubdivCurveAt(src, 0.5)
		dst[2 * idx] = a
		dst[2 * idx + 1] = b
	}
	return
}

@(private = "file")
InjectSubdivCurveSegments :: proc(
	curves_n: ^[dynamic]curve_struct_float(f32),
	split_flags_n: ^[dynamic]bool,
	at_idx: int,
	segs: []curve_struct_float(f32),
) -> (
	added: int,
	err: shape_error = nil,
) {
	if len(segs) == 0 do return

	curves_n[at_idx] = segs[0]
	split_flags_n[at_idx] = true
	if len(segs) > 1 {
		utils_private.non_zero_inject_at_elems(curves_n, at_idx + 1, ..segs[1:]) or_return
		for k := 1; k < len(segs); k += 1 {
			utils_private.non_zero_inject_at_elem(split_flags_n, at_idx + k, true) or_return
		}
	}
	added = len(segs) - 1
	return
}

@(private = "file")
ProcessCurveOverlapPair :: proc(
	curves_a: ^[dynamic]curve_struct_float(f32),
	split_flags_a: ^[dynamic]bool,
	i: int,
	curves_b: ^[dynamic]curve_struct_float(f32),
	split_flags_b: ^[dynamic]bool,
	j: int,
	same_contour: bool,
) -> (
	cur_added: int,
	src_added: int,
	overlapped: bool,
	err: shape_error = nil,
) {
	src := curves_b[j]
	cur := curves_a[i]

	src_is_line := src.type == .Line
	cur_is_line := cur.type == .Line
	if src_is_line && cur_is_line do return
	if !CurveChordOverlaps(src, cur) do return
	overlapped = true

	// curve-vs-line: split only the curve side.
	if src_is_line || cur_is_line {
		if src_is_line {
			cur_half0, cur_half1 := SubdivCurveAt(cur, 0.5)
			cur_segs2 := [2]curve_struct_float(f32){cur_half0, cur_half1}
			cur_segs4: [4]curve_struct_float(f32)
			cur_segs8: [8]curve_struct_float(f32)
			cur_segs: []curve_struct_float(f32) = cur_segs2[:]
			line_probe := [1]curve_struct_float(f32){src}

			if CurveChordOverlapsAny(line_probe[:], cur_segs) {
				SubdivCurveSegmentsAtHalf(cur_segs, cur_segs4[:])
				cur_segs = cur_segs4[:]
				if CurveChordOverlapsAny(line_probe[:], cur_segs) {
					SubdivCurveSegmentsAtHalf(cur_segs, cur_segs8[:])
					cur_segs = cur_segs8[:]
				}
			}
			cur_added = InjectSubdivCurveSegments(curves_a, split_flags_a, i, cur_segs) or_return
		} else {
			src_half0, src_half1 := SubdivCurveAt(src, 0.5)
			src_segs2 := [2]curve_struct_float(f32){src_half0, src_half1}
			src_segs4: [4]curve_struct_float(f32)
			src_segs8: [8]curve_struct_float(f32)
			src_segs: []curve_struct_float(f32) = src_segs2[:]
			line_probe := [1]curve_struct_float(f32){cur}

			if CurveChordOverlapsAny(src_segs, line_probe[:]) {
				SubdivCurveSegmentsAtHalf(src_segs, src_segs4[:])
				src_segs = src_segs4[:]
				if CurveChordOverlapsAny(src_segs, line_probe[:]) {
					SubdivCurveSegmentsAtHalf(src_segs, src_segs8[:])
					src_segs = src_segs8[:]
				}
			}

			if same_contour {
				src_added = InjectSubdivCurveSegments(
					curves_a,
					split_flags_a,
					j,
					src_segs,
				) or_return
			} else {
				src_added = InjectSubdivCurveSegments(
					curves_b,
					split_flags_b,
					j,
					src_segs,
				) or_return
			}
		}
		return
	}

	src_half0, src_half1 := SubdivCurveAt(src, 0.5)
	cur_half0, cur_half1 := SubdivCurveAt(cur, 0.5)
	src_segs2 := [2]curve_struct_float(f32){src_half0, src_half1}
	cur_segs2 := [2]curve_struct_float(f32){cur_half0, cur_half1}
	cur_segs1 := [1]curve_struct_float(f32){cur}
	src_segs4: [4]curve_struct_float(f32)
	src_segs8: [8]curve_struct_float(f32)
	cur_segs4: [4]curve_struct_float(f32)
	cur_segs8: [8]curve_struct_float(f32)
	src_segs: []curve_struct_float(f32) = src_segs2[:]
	cur_segs: []curve_struct_float(f32) = cur_segs1[:]

	// refine one side at a time:
	// src(2) -> cur(2) -> src(4) -> cur(4) -> src(8) -> cur(8)
	if CurveChordOverlapsAny(src_segs, cur_segs) {
		cur_segs = cur_segs2[:]

		if CurveChordOverlapsAny(src_segs, cur_segs) {
			SubdivCurveSegmentsAtHalf(src_segs, src_segs4[:])
			src_segs = src_segs4[:]

			if CurveChordOverlapsAny(src_segs, cur_segs) {
				SubdivCurveSegmentsAtHalf(cur_segs, cur_segs4[:])
				cur_segs = cur_segs4[:]

				if CurveChordOverlapsAny(src_segs, cur_segs) {
					SubdivCurveSegmentsAtHalf(src_segs, src_segs8[:])
					src_segs = src_segs8[:]

					if CurveChordOverlapsAny(src_segs, cur_segs) {
						SubdivCurveSegmentsAtHalf(cur_segs, cur_segs8[:])
						cur_segs = cur_segs8[:]
					}
				}
			}
		}
	}

	if same_contour {
		if len(src_segs) > 1 && len(cur_segs) > 1 {
			if j > i {
				src_added = InjectSubdivCurveSegments(
					curves_a,
					split_flags_a,
					j,
					src_segs,
				) or_return
				cur_added = InjectSubdivCurveSegments(
					curves_a,
					split_flags_a,
					i,
					cur_segs,
				) or_return
			} else {
				cur_added = InjectSubdivCurveSegments(
					curves_a,
					split_flags_a,
					i,
					cur_segs,
				) or_return
				src_added = InjectSubdivCurveSegments(
					curves_a,
					split_flags_a,
					j,
					src_segs,
				) or_return
			}
		} else if len(src_segs) > 1 {
			src_added = InjectSubdivCurveSegments(curves_a, split_flags_a, j, src_segs) or_return
		} else if len(cur_segs) > 1 {
			cur_added = InjectSubdivCurveSegments(curves_a, split_flags_a, i, cur_segs) or_return
		}
	} else {
		src_added = InjectSubdivCurveSegments(curves_b, split_flags_b, j, src_segs) or_return
		if len(cur_segs) > 1 {
			cur_added = InjectSubdivCurveSegments(curves_a, split_flags_a, i, cur_segs) or_return
		}
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
	vertList: [dynamic]shape_vertex2d = make([dynamic]shape_vertex2d, context.temp_allocator)
	indList: [dynamic]u32 = make([dynamic]u32, context.temp_allocator)

	shapes_compute_polygon_in :: proc(
		vertList: ^[dynamic]shape_vertex2d,
		indList: ^[dynamic]u32,
		poly: shapes,
	) -> (
		err: shape_error = nil,
	) {
		non_curves := make([dynamic][][2]f32, context.temp_allocator)
		non_curves2: [dynamic][dynamic][2]f32 = make(
			[dynamic][dynamic][2]f32,
			context.temp_allocator,
		)
		curves2: [dynamic][dynamic]curve_struct_float(f32) = make(
			[dynamic][dynamic]curve_struct_float(f32),
			context.temp_allocator,
		)
		overlap_skip2: [dynamic][dynamic]bool = make(
			[dynamic][dynamic]bool,
			context.temp_allocator,
		)
		insert_ar: [dynamic; 2][2]f32

		for node, nidx in poly.nodes {
			if node.color.a > 0 {
				non_zero_resize_dynamic_array(&non_curves2, len(node.pts)) or_return
				non_zero_resize_dynamic_array(&curves2, len(node.pts)) or_return
				non_zero_resize_dynamic_array(&overlap_skip2, len(node.pts)) or_return

				for i in 0 ..< len(node.pts) {
					if non_curves2[i] == nil do non_curves2[i] = make([dynamic][2]f32, context.temp_allocator) or_return
					if curves2[i] == nil do curves2[i] = make([dynamic]curve_struct_float(f32), context.temp_allocator) or_return
					if overlap_skip2[i] == nil do overlap_skip2[i] = make([dynamic]bool, context.temp_allocator) or_return

					non_zero_resize_dynamic_array(&non_curves2[i], 0) or_return
					non_zero_resize_dynamic_array(&curves2[i], 0) or_return
					non_zero_resize_dynamic_array(&overlap_skip2[i], 0) or_return
				}

				for np, npi in node.pts {
					curve_flags: []bool = nil
					if node.is_curves != nil && npi < len(node.is_curves) {
						curve_flags = node.is_curves[npi]
					}

					if len(np) == 0 do continue

					last := len(np) - 1
					i := 0
					for {
						if !node.is_closed && i >= last do break

						next := node.is_closed ? (i + 1) % len(np) : i + 1
						if next >= len(np) do break

						if next < len(curve_flags) && curve_flags[next] {
							next2 := node.is_closed ? (next + 1) % len(np) : next + 1
							if next2 >= len(np) do break

							if next2 < len(curve_flags) && curve_flags[next2] {
								next3 := node.is_closed ? (next2 + 1) % len(np) : next2 + 1
								if next3 >= len(np) do break

								non_zero_append(
									&curves2[npi],
									curve_struct_float(f32) {
										start = np[i],
										ctl0 = np[next],
										ctl1 = np[next2],
										end = np[next3],
										type = .Unknown,
									},
								) or_return
								i = next3
							} else {
								non_zero_append(
									&curves2[npi],
									curve_struct_float(f32) {
										start = np[i],
										ctl0 = np[next],
										end = np[next2],
										type = .Quadratic,
									},
								) or_return
								i = next2
							}
						} else {
							non_zero_append(
								&curves2[npi],
								curve_struct_float(f32) {
									start = np[i],
									end = np[next],
									type = .Line,
								},
							) or_return
							i = next
						}

						if node.is_closed && i == 0 do break
					}
				}

				// Loop subdivision (float)
				for npi in 0 ..< len(curves2) {
					curves_npi := &curves2[npi]
					for i := 0; i < len(curves_npi); i += 1 {
						c := curves_npi[i]
						if c.type != .Quadratic && c.type != .Line {
							curveType, d0, d1, d2 := GetCubicCurveType(
								c.start,
								c.ctl0,
								c.ctl1,
								c.end,
							) or_return
							if curveType == .Loop {
								disc := 4 * d0 * d2 - 3 * d1 * d1
								t1 := math.sqrt_f32(disc)
								ls := d1 - t1
								lt := 2 * d0
								ms := d1 + t1
								mt := lt

								ql := ls / lt
								qm := ms / mt

								subdiv_at: f32
								if 0.0 < ql && ql < 1.0 {
									subdiv_at = ql
								} else if 0.0 < qm && qm < 1.0 {
									subdiv_at = qm
								} else {
									continue
								}

								SubdivCurveAndInjectAt(
									curves_npi,
									i + 1,
									&curves_npi[i],
									c,
									subdiv_at,
								) or_return
								i += 1
							}
						}
					}
				}

				for npi in 0 ..< len(curves2) {
					skip_npi := &overlap_skip2[npi]
					resize_dynamic_array(skip_npi, len(curves2[npi])) or_return // false by default
				}

				// Overlap check (float)
				// Scan all contour pairs (both directions). Skip logic handles repeat work.
				for npi_a in 0 ..< len(curves2) {
					curves_a := &curves2[npi_a]
					skip_a := &overlap_skip2[npi_a]

					for npi_b := 0; npi_b < len(curves2); npi_b += 1 {
						curves_b := &curves2[npi_b]
						skip_b := &overlap_skip2[npi_b]
						same_contour := npi_a == npi_b

						for i := 0; i < len(curves_a); i += 1 {
							if skip_a[i] do continue

							for j := 0; j < len(curves_b); j += 1 {
								if i >= len(curves_a) do break
								if skip_a[i] do break
								if skip_b[j] do continue
								if same_contour && i == j do continue

								orig_i, orig_j := i, j
								cur_added, src_added, overlapped := ProcessCurveOverlapPair(
									curves_a,
									skip_a,
									i,
									curves_b,
									skip_b,
									j,
									same_contour,
								) or_return
								if !overlapped do continue

								if same_contour {
									if src_added > 0 && orig_j < orig_i do i += src_added
									if cur_added > 0 && orig_i < orig_j do j += cur_added
								}

								if cur_added > 0 do i += cur_added
								if src_added > 0 do j += src_added
								if i >= len(curves_a) do break
							}
						}
					}
				}

				for npi in 0 ..< len(node.pts) {
					non_curves_npi := &non_curves2[npi]
					curves_npi := curves2[npi][:]

					non_zero_resize_dynamic_array(non_curves_npi, 0) or_return
					if len(curves_npi) == 0 do continue

					non_zero_append(non_curves_npi, curves_npi[0].start) or_return
					for c, i in curves_npi {
						if node.is_closed &&
						   i == len(curves_npi) - 1 &&
						   c.end == curves_npi[0].start {
							continue
						}
						non_zero_append(non_curves_npi, c.end) or_return
					}
				}

				// Insert curve control points into polygon boundaries (float)
				for npi in 0 ..< len(node.pts) {
					non_curves_npi := &non_curves2[npi]
					curves_npi := curves2[npi]
					poly_pts := non_curves_npi[:]
					opp_poly_pts: [][2]f32 = nil
					if linalg_ex.GetPolygonOrientation(poly_pts) == .Clockwise {
						for &pts in non_curves2 {
							if &pts == &non_curves2[npi] do continue
							if linalg_ex.PointInPolygon(poly_pts[0], pts[:]) == .Inside {
								opp_poly_pts = pts[:]
								break
							}
						}
					}
					for c in curves_npi {
						if c.type != .Line {
							non_zero_resize_fixed_capacity_dynamic_array(&insert_ar, 0)

							if linalg_ex.PointInPolygon(c.ctl0, poly_pts) == .Inside {
								append_fixed_capacity_elem(&insert_ar, c.ctl0)
							} else if opp_poly_pts != nil &&
							   linalg_ex.PointInPolygon(c.ctl0, opp_poly_pts) == .Inside {
								append_fixed_capacity_elem(&insert_ar, c.ctl0)
							}
							if c.type != .Quadratic {
								if linalg_ex.PointInPolygon(c.ctl1, poly_pts) == .Inside {
									append_fixed_capacity_elem(&insert_ar, c.ctl1)
								} else if opp_poly_pts != nil &&
								   linalg_ex.PointInPolygon(c.ctl1, opp_poly_pts) == .Inside {
									append_fixed_capacity_elem(&insert_ar, c.ctl1)
								}
							}

							if len(insert_ar) > 0 {
								insert_idx := -1
								for idx := 0; idx < len(non_curves_npi); idx += 1 {
									next := idx + 1
									if node.is_closed {
										next %= len(non_curves_npi)
									} else if next >= len(non_curves_npi) {
										break
									}

									if non_curves_npi[idx] == c.start &&
									   non_curves_npi[next] == c.end {
										insert_idx = next
										break
									}
								}
								if insert_idx < 0 {
									continue
								}
								utils_private.non_zero_inject_at_elems(
									non_curves_npi,
									insert_idx,
									..insert_ar[:],
								) or_return
								poly_pts = non_curves_npi[:]
							}
						}
					}
				}

				non_zero_resize_dynamic_array(&non_curves, len(non_curves2)) or_return
				for npi in 0 ..< len(non_curves2) do non_curves[npi] = non_curves2[npi][:]

				indices, tri_err := triangulation.TrianguatePolygons(
					non_curves[:],
					context.temp_allocator,
					auto_cast len(indList^),
				)
				if tri_err != nil {
					switch e in tri_err {
					case triangulation.__Trianguate_Error:
						return e
					case runtime.Allocator_Error:
						return e
					}
				}
				non_zero_append(indList, ..indices) or_return

				for v, i in non_curves {
					for vv in v {
						non_zero_append(
							vertList,
							shape_vertex2d {
								pos = linalg.Vector2f32{vv.x, vv.y},
								flag = .LINE,
								color = node.color,
							},
						) or_return
					}
				}

				for c, i in curves2 {
					for cc in c {
						_Shapes_ComputeLine(vertList, indList, node.color, cc) or_return
					}
				}
			}
		}
		return
	}

	shapes_compute_polygon_in(&vertList, &indList, poly) or_return

	res.vertices = utils_private.make_non_zeroed_slice(
		[]shape_vertex2d,
		len(vertList),
		allocator,
	) or_return
	res.indices = utils_private.make_non_zeroed_slice([]u32, len(indList), allocator) or_return
	mem.copy_non_overlapping(
		raw_data(res.vertices),
		raw_data(vertList[:]),
		len(vertList) * size_of(shape_vertex2d),
	)
	mem.copy_non_overlapping(
		raw_data(res.indices),
		raw_data(indList[:]),
		len(indList) * size_of(u32),
	)
	return
}

poly_transform_matrix :: proc "contextless" (inout_poly: ^shapes, F: linalg.Matrix4x4f32) {
	for &node in inout_poly.nodes {
		for pts in node.pts {
			for &pt in pts {
				out := linalg.mul(F, linalg.Vector4f32{pt.x, pt.y, 0.0, 1.0})
				pt = linalg.Vector2f32{out.x, out.y} / out.w
			}
		}
	}
}
