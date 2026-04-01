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

		EP: T = linalg_ex.epsilon(T) * T(10.0)
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
		if linalg_ex.GetPolygonOrientation([][2]f32{pts.start, pts.ctl0, pts.end}) ==
		   .CounterClockwise {
			non_zero_append(
				vertList,
				shape_vertex2d {
					uvw = {0.0, 0.0, -5.0},
					pos = linalg.Vector2f32{pts.start.x, pts.start.y},
					color = color,
				}, //-5 check this is not cubic curve
			)
			non_zero_append(
				vertList,
				shape_vertex2d {
					uvw = {-0.5, 0.0, -5.0},
					pos = linalg.Vector2f32{pts.ctl0.x, pts.ctl0.y},
					color = color,
				},
			)
			non_zero_append(
				vertList,
				shape_vertex2d {
					uvw = {-1.0, -1.0, -5.0},
					pos = linalg.Vector2f32{pts.end.x, pts.end.y},
					color = color,
				},
			)
		} else {
			non_zero_append(
				vertList,
				shape_vertex2d {
					uvw = {0.0, 0.0, -5.0},
					pos = linalg.Vector2f32{pts.start.x, pts.start.y},
					color = color,
				},
			)
			non_zero_append(
				vertList,
				shape_vertex2d {
					uvw = {0.5, 0.0, -5.0},
					pos = linalg.Vector2f32{pts.ctl0.x, pts.ctl0.y},
					color = color,
				},
			)
			non_zero_append(
				vertList,
				shape_vertex2d {
					uvw = {1.0, 1.0, -5.0},
					pos = linalg.Vector2f32{pts.end.x, pts.end.y},
					color = color,
				},
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

		reverse =
			(d0 > 0 && F[1][0] < math.F32_EPSILON * 5.0) ||
			(d0 < 0 && F[1][0] > -math.F32_EPSILON * 5.0)
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
			{{}, {}, {}},
			{1.0 / 3.0, {}, 1.0 / 3.0},
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
		insert_ar := make([dynamic][2]f32, context.temp_allocator)

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
				p0, p1_, p2_ := linalg_ex.SubdivQuadraticBezier(
					[3][2]f32{src.start, src.ctl0, src.end},
					t,
				)
				cur.ctl0, cur.end = p0, p1_
				mid, c0 = p1_, p2_
				utils_private.non_zero_inject_at_elem(
					curves_n,
					insert_idx,
					curve_struct_float(f32) {
						start = p1_,
						ctl0 = p2_,
						end = cur.end,
						type = .Quadratic,
					},
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
						end = cur.end,
						type = .Unknown,
					},
				) or_return
			}
			return
		}

		for node, nidx in poly.nodes {
			if node.color.a > 0 {
				non_zero_resize_dynamic_array(&non_curves2, len(node.pts)) or_return
				non_zero_resize_dynamic_array(&curves2, len(node.pts)) or_return

				for i in 0 ..< len(node.pts) {
					if non_curves2[i] == nil {
						non_curves2[i] = make([dynamic][2]f32, context.temp_allocator) or_return
						curves2[i] = make(
							[dynamic]curve_struct_float(f32),
							context.temp_allocator,
						) or_return
					} else {
						non_zero_resize_dynamic_array(&non_curves2[i], 0) or_return
						non_zero_resize_dynamic_array(&curves2[i], 0) or_return
					}
				}

				for np, npi in node.pts {
					for i := 0; i < len(np); i += 1 {
						pt := np[i]
						if node.is_curves != nil &&
						   npi < len(node.is_curves) &&
						   i < len(node.is_curves[npi]) &&
						   node.is_curves[npi][i] {
							if i + 1 < len(node.is_curves[npi]) && node.is_curves[npi][i + 1] {
								non_zero_append(
									&curves2[npi],
									curve_struct_float(f32) {
										start = np[i - 1],
										ctl0 = pt,
										ctl1 = np[i + 1],
										end = np[(i + 2) % len(np)],
										type = .Unknown,
									},
								) or_return
								i += 1
							} else {
								non_zero_append(
									&curves2[npi],
									curve_struct_float(f32) {
										start = np[i - 1],
										ctl0 = pt,
										end = np[(i + 1) % len(np)],
										type = .Quadratic,
									},
								) or_return
							}
						} else {
							non_zero_append(&non_curves2[npi], pt) or_return
						}
					}
				}

				// Loop subdivision (float)
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
								disc := 4 * d0 * d2 - 3 * d1 * d1
								// Due to float error, disc can be a tiny negative even for Loop.
								t1: f32
								if disc <= 0 {
									if disc > -math.F32_EPSILON do disc = 0.0
									else do continue
									t1 = 0.0
								} else {
									t1 = math.sqrt_f32(disc)
								}
								ls := d1 - t1
								lt := 2 * d0
								ms := d1 + t1
								mt := lt

								subdiv_at: f32
								if lt > 0 ? (0 < ls && ls < lt) : (0 > ls && ls > lt) {
									subdiv_at = ls / lt
								} else if mt > 0 ? (0 < ms && ms < mt) : (0 > ms && ms > mt) {
									subdiv_at = ms / mt
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

				// Overlap check (float)
				overlap_skip2: [][dynamic]bool = utils_private.make_non_zeroed_slice(
					[][dynamic]bool,
					len(curves2),
					context.temp_allocator,
				) or_return
				for &ov in overlap_skip2 do ov = make([dynamic]bool, context.temp_allocator) or_return

				for npi in 0 ..< len(curves2) {
					curves_n := &curves2[npi]
					skip_n := &overlap_skip2[npi]
					non_zero_resize(skip_n, len(curves_n)) or_return

					for i := 0; i < len(curves_n); i += 1 {
						for j := i + 1; j < len(curves_n); j += 1 {
							if skip_n[i] || skip_n[j] do continue
							cur := curves_n[i]
							cur2 := curves_n[j]

							k1, ip1 := linalg_ex.LinesIntersect2(
								cur2.start,
								cur2.end,
								cur.start,
								cur.ctl0,
								true,
							)
							if k1 == .intersect {
								_, t := linalg_ex.PointInLine(ip1, cur2.start, cur2.end)
								non_zero_resize(skip_n, len(skip_n) + 1) or_return
								skip_n[i], skip_n[j], skip_n[j + 1] = true, true, true

								j += 1
								mid, c0, c1 := SubdivCurveAndInjectAt(
									curves_n,
									j,
									&cur2,
									cur2,
									t,
								) or_return

								// opp
								k2, ip2 := linalg_ex.LinesIntersect2(
									mid,
									cur2.end,
									cur.type == .Quadratic ? cur.ctl0 : cur.ctl1,
									cur.end,
									true,
								)
								if k2 != .intersect do continue
								non_zero_resize(skip_n, len(skip_n) + 1) or_return
								_, t2 := linalg_ex.PointInLine(ip2, mid, cur2.end)

								if cur2.type == .Quadratic {
									skip_n[j + 1] = true
									SubdivCurveAndInjectAt(
										curves_n,
										j + 1,
										&curves_n[j],
										curve_struct_float(f32) {
											start = mid,
											ctl0 = c0,
											end = cur2.end,
											type = .Quadratic,
										},
										t2,
									) or_return
									j += 1
								} else {
									SubdivCurveAndInjectAt(
										curves_n,
										j + 1,
										&curves_n[j],
										curve_struct_float(f32) {
											start = mid,
											ctl0 = c0,
											ctl1 = c1,
											end = cur2.end,
											type = .Unknown,
										},
										t2,
									) or_return
								}
							} else {
								k2, ip2 := linalg_ex.LinesIntersect2(
									cur2.start,
									cur2.end,
									cur.type == .Quadratic ? cur.ctl0 : cur.ctl1,
									cur.end,
									true,
								)
								if k2 != .intersect do continue
								non_zero_resize(skip_n, len(skip_n) + 1) or_return
								skip_n[i], skip_n[j], skip_n[j + 1] = true, true, true
								j += 1

								_, t := linalg_ex.PointInLine(ip2, cur2.start, cur2.end)
								SubdivCurveAndInjectAt(curves_n, j, &cur2, cur2, t) or_return
							}
						}
					}
				}

				// Insert curve control points into polygon boundaries (float)
				for npi in 0 ..< len(node.pts) {
					non_curves_npi := &non_curves2[npi]
					curves_npi := curves2[npi]
					curve_idx := 0
					poly_pts := non_curves_npi[:]
					orient := linalg_ex.GetPolygonOrientation(poly_pts)
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
							non_zero_resize_dynamic_array(&insert_ar, 0) or_return

							if ok :=
								   linalg_ex.PointInPolygon(
									   curves_npi[curve_idx].ctl0,
									   poly_pts,
								   ) ==
								   .Inside; invent_b ? !ok : ok {
								non_zero_append(&insert_ar, curves_npi[curve_idx].ctl0) or_return
							}
							if curves_npi[curve_idx].type != .Quadratic {
								if ok :=
									   linalg_ex.PointInPolygon(
										   curves_npi[curve_idx].ctl1,
										   poly_pts,
									   ) ==
									   .Inside; invent_b ? !ok : ok {
									non_zero_append(
										&insert_ar,
										curves_npi[curve_idx].ctl1,
									) or_return
								}
							}

							utils_private.non_zero_inject_at_elems(
								non_curves_npi,
								next,
								..insert_ar[:],
							) or_return
							poly_pts = non_curves_npi[:]

							curve_idx += 1
							i = next + len(insert_ar)
						} else {
							i += 1
						}
					}
				}

				non_zero_resize_dynamic_array(&non_curves, len(non_curves2)) or_return
				for npi in 0 ..< len(non_curves2) do non_curves[npi] = non_curves2[npi][:]

				//fmt.println(non_curves[:])
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
								uvw = {0.0, 0.0, -100.0},
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

