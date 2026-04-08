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
	LINE_EDGE,
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
curve_type :: enum u8 {
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
	Length_Mismatch,
	Empty_Input,
	First_Point_Is_Curve,
	Consecutive_Anchor_Missing_Control,
	Too_Many_Consecutive_Curves,
	OverFlow,
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

@(private)
FindParentContour :: proc(idx: int, pts_in: [][]linalg.Vector2f32) -> int {
	if idx < 0 || idx >= len(pts_in) do return -1
	if len(pts_in[idx]) < 3 do return -1

	p := pts_in[idx][0]

	parent_idx := -1
	parent_area := f32(0)

	for j in 0 ..< len(pts_in) {
		if j == idx do continue

		other := pts_in[j]
		if len(other) < 3 do continue

		if linalg_ex.PointInPolygon(p, other) == .Outside {
			continue
		}

		// idx를 포함하는 contour 중 가장 작은 면적의 contour를 parent로 선택
		area := abs(linalg_ex.PolygonSignedArea(other))

		if parent_idx == -1 || area < parent_area {
			parent_idx = j
			parent_area = area
		}
	}

	return parent_idx
}

IsHoleContour :: proc(idx: int, pts_in: [][]linalg.Vector2f32) -> bool {
	if idx < 0 || idx >= len(pts_in) do return false
	if len(pts_in[idx]) < 3 do return false

	parent := FindParentContour(idx, pts_in)
	if parent == -1 {
		// parent가 없으면 최외곽 contour
		return false
	}

	// parent가 outer면 나는 hole
	// parent가 hole이면 나는 outer
	return !IsHoleContour(parent, pts_in)
}

ReverseShapeCloseCurve :: proc(
	pts: []linalg.Vector2f32,
	is_curves: []bool,
	allocator := context.allocator,
) -> (
	out_pts: []linalg.Vector2f32,
	out_is_curves: []bool,
	err: shape_error,
) {
	if len(pts) > int(max(u32)) || len(is_curves) > int(max(u32)) {
		return nil, nil, .OverFlow
	}

	n := u32(len(pts))

	if n != u32(len(is_curves)) {
		return nil, nil, .Length_Mismatch
	}
	if n == 0 {
		return nil, nil, .Empty_Input
	}
	if is_curves[0] {
		return nil, nil, .First_Point_Is_Curve
	}

	anchor_indices := make([dynamic]u32, context.temp_allocator) or_return

	for i in 0 ..< n {
		if !is_curves[i] {
			non_zero_append(&anchor_indices, i) or_return
		}
	}

	if len(anchor_indices) == 0 {
		return nil, nil, .First_Point_Is_Curve
	}

	// 각 anchor 사이 control 개수 검사
	for a := 0; a < len(anchor_indices); a += 1 {
		start := anchor_indices[a]
		next := anchor_indices[(a + 1) % len(anchor_indices)]

		curve_count := 0
		i := (start + 1) % n
		for i != next {
			if !is_curves[i] {
				return nil, nil, .Consecutive_Anchor_Missing_Control
			}
			curve_count += 1
			i = (i + 1) % n
		}

		if curve_count == 0 {
			return nil, nil, .Consecutive_Anchor_Missing_Control
		}
		if curve_count > 2 {
			return nil, nil, .Too_Many_Consecutive_Curves
		}
	}

	out_pts = utils_private.make_non_zeroed_slice([]linalg.Vector2f32, n, allocator) or_return
	defer if err != nil do delete(out_pts, allocator)

	out_is_curves = utils_private.make_non_zeroed_slice([]bool, n, allocator) or_return
	defer if err != nil do delete(out_is_curves, allocator)

	out_pts[0] = pts[0]
	out_is_curves[0] = false
	out_i: u32 = 1

	m := len(anchor_indices)

	// reverse된 순서대로 모든 세그먼트를 처리
	// step=0: last -> first
	// step=1: second_last -> last
	// ...
	// step=m-1: first -> second
	for step := 0; step < m; step += 1 {
		end_anchor_pos := (m - step) % m
		start_anchor_pos := (end_anchor_pos - 1 + m) % m

		a0 := anchor_indices[start_anchor_pos]
		a1 := anchor_indices[end_anchor_pos]

		controls := [2]u32{}
		curve_count := 0

		i := (a0 + 1) % n
		for i != a1 {
			controls[curve_count] = i
			curve_count += 1
			i = (i + 1) % n
		}

		switch curve_count {
		case 1:
			out_pts[out_i] = pts[controls[0]]
			out_is_curves[out_i] = true
			out_i += 1

		case 2:
			out_pts[out_i] = pts[controls[1]]
			out_is_curves[out_i] = true
			out_i += 1

			out_pts[out_i] = pts[controls[0]]
			out_is_curves[out_i] = true
			out_i += 1

		case:
			return nil, nil, .Too_Many_Consecutive_Curves
		}

		// 마지막 세그먼트는 first anchor로 닫히므로
		// anchor 0은 다시 쓰지 않음
		if a0 != 0 {
			out_pts[out_i] = pts[a0]
			out_is_curves[out_i] = false
			out_i += 1
		}
	}

	if out_i != n {
		return nil, nil, .EmptyPolygon
	}

	return out_pts, out_is_curves, nil
}

// Same layout as curve_struct_fixed for floating-point coordinates (f16 / f32 / f64).
@(private)
curve_struct_float :: struct($F: typeid) where intrinsics.type_is_float(F) {
	start:         [2]F,
	ctl0:          [2]F,
	ctl1:          [2]F,
	end:           [2]F,
	type:          curve_type,
	curve_reverse: bool,
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
) where intrinsics.type_is_float(T) {
	if start == control0 && control0 == control1 && control1 == end {
		err = .IsPointNotLine
		return
	}
	start: [2]f64 = [2]f64{f64(start.x), f64(start.y)}
	control0: [2]f64 = [2]f64{f64(control0.x), f64(control0.y)}
	control1: [2]f64 = [2]f64{f64(control1.x), f64(control1.y)}
	end: [2]f64 = [2]f64{f64(end.x), f64(end.y)}

	cross_1 := [3]f64 {
		end.y - control1.y,
		control1.x - end.x,
		end.x * control1.y - end.y * control1.x,
	}
	cross_2 := [3]f64{start.y - end.y, end.x - start.x, start.x * end.y - start.y * end.x}
	cross_3 := [3]f64 {
		control0.y - start.y,
		start.x - control0.x,
		control0.x * start.y - control0.y * start.x,
	}

	a1 := start.x * cross_1.x + start.y * cross_1.y + cross_1.z //9
	a2 := control0.x * cross_2.x + control0.y * cross_2.y + cross_2.z //7
	a3 := control1.x * cross_3.x + control1.y * cross_3.y + cross_3.z //7

	d0_: f64 = a1 - 2.0 * a2 + 3.0 * a3 //27
	d1_: f64 = -a2 + 3.0 * a3 //16
	d2_: f64 = 3.0 * a3 //8

	D := 3.0 * d1_ * d1_ - 4.0 * d2_ * d0_ //33 + 36 + 1 = 70
	discr := d0_ * d0_ * D //27 + 27 + 70 = 124

	EP: f64 = linalg_ex.epsilon(f64)
	d0 = T(d0_)
	d1 = T(d1_)
	d2 = T(d2_)
	if discr >= -EP && discr <= EP {
		if d0_ == 0.0 && d1_ == 0.0 {
			if d2_ == 0.0 {
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

@(private = "file")
_Shapes_ComputeLine :: proc(
	vertList: ^[dynamic]shape_vertex2d,
	indList: ^[dynamic]u32,
	color: linalg.Vector4f32,
	pts: ^curve_struct_float(f32), //inout
) -> shape_error {
	curveType := pts.type
	err: shape_error = nil
	//loop_reverse := false
	//if curveType == .LoopReverse do loop_reverse = true

	pts.curve_reverse = false
	reverse := false
	d0, d1, d2: f32
	if curveType != .Line && curveType != .Quadratic {
		curveType, d0, d1, d2 = GetCubicCurveType(pts.start, pts.ctl0, pts.ctl1, pts.end) or_return
		//pts.type = curveType
	} else if curveType == .Quadratic {
		vlen: u32 = u32(len(vertList))
		quad_sign := f32(1.0)
		if linalg_ex.GetPolygonOrientation([][2]f32{pts.start, pts.ctl0, pts.end}) ==
		   .CounterClockwise {
			quad_sign = -1.0
			pts.curve_reverse = true
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
		t1: f32 = math.sqrt_f32(4 * d0 * d2 - 3 * d1 * d1)
		ls := d1 - t1
		lt := 2 * d0
		ms := d1 + t1
		mt := lt

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
	pts.curve_reverse = reverse
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
	appendLine(vertList, indList, color, pts^, F) or_return

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
				for c, i in curves2 {
					for &cc in c {
						_Shapes_ComputeLine(vertList, indList, node.color, &cc) or_return
					}
				}

				// Insert curve control points into polygon boundaries (float)
				for npi in 0 ..< len(node.pts) {
					non_curves_npi := &non_curves2[npi]
					curves_npi := curves2[npi]
					poly_pts := non_curves_npi[:]

					for c in curves_npi {
						if c.type != .Line {
							non_zero_resize_fixed_capacity_dynamic_array(&insert_ar, 0)

							if c.type != .Quadratic {
								if linalg_ex.PointLineLeftOrRight(c.ctl0, c.start, c.end) > 0 {
									append_fixed_capacity_elem(&insert_ar, c.ctl0)
								}
								if linalg_ex.PointLineLeftOrRight(c.ctl1, c.start, c.end) > 0 {
									append_fixed_capacity_elem(&insert_ar, c.ctl1)
								}
							} else {
								if linalg_ex.PointLineLeftOrRight(c.ctl0, c.start, c.end) > 0 {
									append_fixed_capacity_elem(&insert_ar, c.ctl0)
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

				contour_edge_count :: proc "contextless" (
					contour: [][2]f32,
					curves: []curve_struct_float(f32),
				) -> (
					edge_count: int,
					is_closed: bool,
				) {
					n := len(contour)
					if n < 2 do return
					// non_curves build rule:
					// closed contour => len(curves) == len(contour)
					// open contour   => len(curves) == len(contour) - 1
					is_closed = len(curves) == n
					edge_count = n - 1
					if is_closed do edge_count = n
					return
				}

				append_line_triangle :: proc(
					vertList: ^[dynamic]shape_vertex2d,
					indList: ^[dynamic]u32,
					color: linalg.Vector4f32,
					p0, p1, p2: linalg.Vector2f32,
					apply_edge_aa: bool,
				) -> shape_error {
					start := u32(len(vertList^))
					flag: shape_vertex_flag = .LINE
					uv0 := linalg.Vector3f32{0.0, 0.0, 0.0}
					uv1 := linalg.Vector3f32{0.0, 0.0, 0.0}
					uv2 := linalg.Vector3f32{0.0, 0.0, 0.0}

					if apply_edge_aa {
						flag = .LINE_EDGE
						uv0.x = 0.0
						uv1.x = 0.0
						uv2.x = 1.0
					}

					non_zero_append(
						vertList,
						shape_vertex2d{pos = p0, uvw = uv0, color = color, flag = flag},
						shape_vertex2d{pos = p1, uvw = uv1, color = color, flag = flag},
						shape_vertex2d{pos = p2, uvw = uv2, color = color, flag = flag},
					) or_return
					non_zero_append(indList, start, start + 1, start + 2) or_return
					return nil
				}

				flat_pts := make([dynamic]linalg.Vector2f32, context.temp_allocator) or_return
				total_pts := 0
				for contour in non_curves do total_pts += len(contour)
				non_zero_reserve(&flat_pts, total_pts) or_return

				boundary_edges: [][]bool = make(
					[][]bool,
					total_pts,
					context.temp_allocator,
				) or_return
				for i := 0; i < total_pts; i += 1 {
					boundary_edges[i] = make([]bool, total_pts, context.temp_allocator) or_return
				}

				base_idx: u32 = 0
				for contour, npi in non_curves {
					for p in contour {
						non_zero_append(&flat_pts, linalg.Vector2f32{p.x, p.y}) or_return
					}

					curves_npi := curves2[npi][:]
					edge_count, is_closed := contour_edge_count(contour, curves_npi)
					if edge_count <= 0 {
						base_idx += u32(len(contour))
						continue
					}

					for c in curves_npi {
						if c.type != .Line do continue

						// mark all matching contour edges.
						// This avoids misses when same coordinates appear at multiple indices.
						for i := 0; i < edge_count; i += 1 {
							next := i + 1
							if is_closed && next == len(contour) do next = 0
							if contour[i] != c.start || contour[next] != c.end do continue

							a := int(base_idx) + i
							b := int(base_idx) + next
							if a == b do continue
							boundary_edges[a][b] = true
							boundary_edges[b][a] = true
						}
					}

					base_idx += u32(len(contour))
				}

				indices, tri_err := triangulation.TrianguatePolygons(
					non_curves[:],
					context.temp_allocator,
				)
				if tri_err != nil {
					switch e in tri_err {
					case triangulation.__Trianguate_Error:
						return e
					case runtime.Allocator_Error:
						return e
					}
				}

				for t := 0; t + 2 < len(indices); t += 3 {
					ia := indices[t]
					ib := indices[t + 1]
					ic := indices[t + 2]

					pa := flat_pts[ia]
					pb := flat_pts[ib]
					pc := flat_pts[ic]

					ab := boundary_edges[ia][ib]
					bc := boundary_edges[ib][ic]
					ca := boundary_edges[ic][ia]

					boundary_count := 0
					if ab do boundary_count += 1
					if bc do boundary_count += 1
					if ca do boundary_count += 1

					if boundary_count == 0 {
						append_line_triangle(
							vertList,
							indList,
							node.color,
							pa,
							pb,
							pc,
							false,
						) or_return
						continue
					}

					if boundary_count == 1 {
						if ab {
							append_line_triangle(
								vertList,
								indList,
								node.color,
								pa,
								pb,
								pc,
								true,
							) or_return
						} else if bc {
							append_line_triangle(
								vertList,
								indList,
								node.color,
								pb,
								pc,
								pa,
								true,
							) or_return
						} else {
							append_line_triangle(
								vertList,
								indList,
								node.color,
								pc,
								pa,
								pb,
								true,
							) or_return
						}
						continue
					}

					if boundary_count == 2 {
						if ab && bc {
							// shared vertex: b, split opposite edge a-c
							mid := linalg.Vector2f32 {
								(pa.x + pc.x) * 0.5,
								(pa.y + pc.y) * 0.5,
							}
							append_line_triangle(
								vertList,
								indList,
								node.color,
								pa,
								pb,
								mid,
								true,
							) or_return
							append_line_triangle(
								vertList,
								indList,
								node.color,
								pb,
								pc,
								mid,
								true,
							) or_return
						} else if bc && ca {
							// shared vertex: c, split opposite edge a-b
							mid := linalg.Vector2f32 {
								(pa.x + pb.x) * 0.5,
								(pa.y + pb.y) * 0.5,
							}
							append_line_triangle(
								vertList,
								indList,
								node.color,
								pb,
								pc,
								mid,
								true,
							) or_return
							append_line_triangle(
								vertList,
								indList,
								node.color,
								pc,
								pa,
								mid,
								true,
							) or_return
						} else {
							// shared vertex: a, split opposite edge b-c
							mid := linalg.Vector2f32 {
								(pb.x + pc.x) * 0.5,
								(pb.y + pc.y) * 0.5,
							}
							append_line_triangle(
								vertList,
								indList,
								node.color,
								pc,
								pa,
								mid,
								true,
							) or_return
							append_line_triangle(
								vertList,
								indList,
								node.color,
								pa,
								pb,
								mid,
								true,
							) or_return
						}
						continue
					}

					center := linalg.Vector2f32 {
						(pa.x + pb.x + pc.x) / 3.0,
						(pa.y + pb.y + pc.y) / 3.0,
					}
					append_line_triangle(
						vertList,
						indList,
						node.color,
						pa,
						pb,
						center,
						ab,
					) or_return
					append_line_triangle(
						vertList,
						indList,
						node.color,
						pb,
						pc,
						center,
						bc,
					) or_return
					append_line_triangle(
						vertList,
						indList,
						node.color,
						pc,
						pa,
						center,
						ca,
					) or_return
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
