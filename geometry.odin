package geometry

import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:math/linalg"
import "core:mem"

import "core:math/fixed"

//Cover MAX 741455 * 741455
@private FIXED_SHIFT :: 24
FixedDef :: fixed.Fixed(i64, FIXED_SHIFT)
@private OneFixed :: fixed.Fixed(i64, FIXED_SHIFT){i = 1 << FIXED_SHIFT}
@private TwoFixed :: fixed.Fixed(i64, FIXED_SHIFT){i = 2 << FIXED_SHIFT}
@private ThreeFixed :: fixed.Fixed(i64, FIXED_SHIFT){i = 3 << FIXED_SHIFT}
@private Div3Fixed :: fixed.Fixed(i64, FIXED_SHIFT){i = (2 << FIXED_SHIFT) / 3}
@private Div3Mul2Fixed :: fixed.Fixed(i64, FIXED_SHIFT){i = (1 << FIXED_SHIFT) / 3}

shape_vertex2di64 :: struct #align(1) {
    pos: [2]FixedDef,
    uvw: linalg.point3d,
    color: linalg.point3dw,
};

shape_vertex2d :: struct #align(1) {
    pos: linalg.point,
    uvw: linalg.point3d,
    color: linalg.point3dw,
};

raw_shape :: struct {
    vertices : []shape_vertex2d,
    indices:[]u32,
}

raw_shapei64 :: struct {
    vertices : []shape_vertex2di64,
    indices:[]u32,
}

@private curve_type :: enum {
    Line,
    Unknown,
    Serpentine,
    Loop,
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
    runtime.Allocator_Error,
}

shape_node :: struct {
    pts: []linalg.point,
	curve_pts_ids: []u32,
    color: linalg.point3dw,
    stroke_color: linalg.point3dw,
    thickness: f64,
	is_closed: bool,
}

shapes :: struct {
    nodes: []shape_node,
}

shape_nodei64 :: struct {
    pts: [][2]FixedDef,
	n_polys:[]u32,
	curve_pts_ids: []u32,
    color: linalg.point3dw,
    stroke_color: linalg.point3dw,
    thickness: FixedDef,
	is_closed: bool,
}

shapesi64 :: struct {
    nodes: []shape_nodei64,
}


CvtQuadraticToCubic0 :: #force_inline proc "contextless" (_start : linalg.point, _control : linalg.point) -> linalg.point {
    return linalg.point{ _start.x + (2.0/3.0) * (_control.x - _start.x), _start.y + (2.0/3.0) * (_control.y - _start.y) }
}
CvtQuadraticToCubic1 :: #force_inline proc "contextless" (_end : linalg.point, _control : linalg.point) -> linalg.point {
    return CvtQuadraticToCubic0(_end, _control)
}

rect_line_init :: proc "contextless" (_rect: linalg.rect) -> [4]linalg.point {
	return [4]linalg.point{linalg.point{_rect.left, _rect.top}, linalg.point{_rect.left, _rect.bottom}, linalg.point{_rect.right, _rect.bottom}, linalg.point{_rect.right, _rect.top}}
}

round_rect_line_init :: proc "contextless" (_rect: linalg.rect, _radius: f32) -> (pts:[16]linalg.point, curve_pts_ids:[8]u32) {
	r := _radius
	// Clamp radius to fit within rect
	half_width := (_rect.right - _rect.left) * 0.5
	half_height := abs(_rect.bottom - _rect.top) * 0.5
	r = min(r, min(half_width, half_height))
	
	t: f32 = (4.0 / 3.0) * math.tan_f32(math.PI / 8.0)
	tt := t * r
	
	// Corner centers
	top_left := linalg.point{_rect.left + r, _rect.top + r}
	top_right := linalg.point{_rect.right - r, _rect.top + r}
	bottom_right := linalg.point{_rect.right - r, _rect.bottom - r}
	bottom_left := linalg.point{_rect.left + r, _rect.bottom - r}
	
	return [16]linalg.point{
		// Top-left corner (cubic) - counter-clockwise: from top to left
		// Note: y increases upward, _rect.top is top (larger y), _rect.bottom is bottom (smaller y)
			linalg.point{_rect.left + r, _rect.top},
			linalg.point{_rect.left + r - tt, _rect.top},
			linalg.point{_rect.left, _rect.top - r + tt},
		// Left line - counter-clockwise: from top to bottom (y decreases)
			linalg.point{_rect.left, _rect.top - r},
		// Bottom-left corner (cubic) - counter-clockwise: from left to bottom
			linalg.point{_rect.left, _rect.bottom + r},
			linalg.point{_rect.left, _rect.bottom + r - tt},
			linalg.point{_rect.left + r - tt, _rect.bottom},
		// Bottom line - counter-clockwise: from left to right
			linalg.point{_rect.left + r, _rect.bottom},
		// Bottom-right corner (cubic) - counter-clockwise: from bottom to right
			linalg.point{_rect.right - r, _rect.bottom},
			linalg.point{_rect.right - r + tt, _rect.bottom},
			linalg.point{_rect.right, _rect.bottom + r - tt},
		// Right line - counter-clockwise: from bottom to top (y increases)
			linalg.point{_rect.right, _rect.bottom + r},
		// Top-right corner (cubic) - counter-clockwise: from right to top
			linalg.point{_rect.right, _rect.top - r},
			linalg.point{_rect.right, _rect.top - r + tt},
			linalg.point{_rect.right - r + tt, _rect.top},
		// Top line - counter-clockwise: from right to left
			linalg.point{_rect.right - r, _rect.top},
	}, [8]u32{1,2, 5,6, 9,10, 13,14}
}

circle_cubic_init :: proc "contextless" (_center: linalg.point, _r: f32) -> (pts:[12]linalg.point, curve_pts_ids:[8]u32) {
	t: f32 = (4.0 / 3.0) * math.tan_f32(math.PI / 8.0)
	tt := t * _r
	return [12]linalg.point{
			linalg.point{_center.x - _r, _center.y},
			linalg.point{_center.x - _r, _center.y - tt},
			linalg.point{_center.x - tt, _center.y - _r},

			linalg.point{_center.x, _center.y - _r},
			linalg.point{_center.x + tt, _center.y - _r},
			linalg.point{_center.x + _r, _center.y - tt},

			linalg.point{_center.x + _r, _center.y},
			linalg.point{_center.x + _r, _center.y + tt},
			linalg.point{_center.x + tt, _center.y + _r},

			linalg.point{_center.x, _center.y + _r},
			linalg.point{_center.x - tt, _center.y + _r},
			linalg.point{_center.x - _r, _center.y + tt},
	}, [8]u32{1,2, 4,5, 7,8, 10,11}
}

ellipse_cubic_init :: proc "contextless" (_center: linalg.point, _rxy: linalg.point) -> (pts:[12]linalg.point, curve_pts_ids:[8]u32) {
	t: f32 = (4.0 / 3.0) * math.tan_f32(math.PI / 8.0)
	ttx := t * _rxy.x
	tty := t * _rxy.y
	return [12]linalg.point{
			linalg.point{_center.x - _rxy.x, _center.y},
			linalg.point{_center.x - _rxy.x, _center.y - tty},
			linalg.point{_center.x - ttx, _center.y - _rxy.y},

			linalg.point{_center.x, _center.y - _rxy.y},
			linalg.point{_center.x + ttx, _center.y - _rxy.y},
			linalg.point{_center.x + _rxy.x, _center.y - tty},

			linalg.point{_center.x + _rxy.x, _center.y},
			linalg.point{_center.x + _rxy.x, _center.y + tty},
			linalg.point{_center.x + ttx, _center.y + _rxy.y},

			linalg.point{_center.x, _center.y + _rxy.y},
			linalg.point{_center.x - ttx, _center.y + _rxy.y},
			linalg.point{_center.x - _rxy.x, _center.y + tty},
	}, [8]u32{1,2, 4,5, 7,8, 10,11}
}

raw_shape_free :: proc (self:raw_shape, allocator := context.allocator) {
    delete(self.vertices, allocator)
    delete(self.indices, allocator)
}

raw_shapei64_free :: proc (self:raw_shapei64, allocator := context.allocator) {
    delete(self.vertices, allocator)
    delete(self.indices, allocator)
}

raw_shapei64_clone :: proc (self:raw_shapei64, allocator := context.allocator) -> (res:raw_shapei64, err: runtime.Allocator_Error) #optional_allocator_error {
    res.vertices = mem.make_non_zeroed_slice([]shape_vertex2di64, len(self.vertices), allocator) or_return
    defer if err != nil do delete(res.vertices, allocator)

    res.indices = mem.make_non_zeroed_slice([]u32, len(self.indices), allocator) or_return

    intrinsics.mem_copy_non_overlapping(&res.vertices[0], &self.vertices[0], len(self.vertices) * size_of(shape_vertex2d))
    intrinsics.mem_copy_non_overlapping(&res.indices[0], &self.indices[0], len(self.indices) * size_of(u32))
    return
}

raw_shape_clone :: proc (self:^raw_shape, allocator := context.allocator) -> (res:^raw_shape = nil, err: runtime.Allocator_Error) #optional_allocator_error {
    res = new(raw_shape, allocator) or_return
    defer if err != nil {
        free(res, allocator)
        res = nil
    }

    res.vertices = mem.make_non_zeroed_slice([]shape_vertex2d, len(self.vertices), allocator) or_return
    defer if err != nil do delete(res.vertices, allocator)

    res.indices = mem.make_non_zeroed_slice([]u32, len(self.indices), allocator) or_return

    intrinsics.mem_copy_non_overlapping(&res.vertices[0], &self.vertices[0], len(self.vertices) * size_of(shape_vertex2d))
    intrinsics.mem_copy_non_overlapping(&res.indices[0], &self.indices[0], len(self.indices) * size_of(u32))
    return
}

GetCubicCurveType :: proc "contextless" (_start:[2]$T, _control0:[2]T, _control1:[2]T, _end:[2]T) ->
(type:curve_type = .Unknown, err:shape_error = nil, outD:[3]T) where intrinsics.type_is_numeric(T) {
    if _start == _control0 && _control0 == _control1 && _control1 == _end {
        err = .IsPointNotLine
        return
    }

    start := [2]T{_start[0], _start[1]}
    c0    := [2]T{_control0[0], _control0[1]}
    c1    := [2]T{_control1[0], _control1[1]}
    end   := [2]T{_end[0], _end[1]}

	when intrinsics.type_is_integer(T) {
		cross_1 := [3]T{end.y - c1.y,     c1.x - end.x,     (end.x * c1.y / PRECISION) - (end.y * c1.x / PRECISION)}
		cross_2 := [3]T{start.y - end.y,  end.x - start.x,  (start.x * end.y / PRECISION) - (start.y * end.x / PRECISION)}
		cross_3 := [3]T{c0.y - start.y,   start.x - c0.x,   (c0.x * start.y / PRECISION) - (c0.y * start.x / PRECISION)}

		a1 := (start.x * cross_1.x / PRECISION)  + (start.y * cross_1.y / PRECISION)  + cross_1.z
		a2 := (c0.x * cross_2.x / PRECISION)   + (c0.y * cross_2.y / PRECISION)       + cross_2.z
		a3 := (c1.x * cross_3.x / PRECISION)   + (c1.y * cross_3.y / PRECISION)       + cross_3.z

		d0 := a1 - 2 * a2 + 3 * a3
		d1 := -a2 + 3 * a3
		d2 := 3 * a3

		outD.x = d0
		outD.y = d1
		outD.z = d2

		D     := 3 * d1 * d1 / PRECISION - 4 * d2 * d0 / PRECISION
		discr := d0 * d0 / PRECISION * D / PRECISION 

		if discr == 0 {
			if d0 == 0 && d1 == 0 {
				if d2 == 0 {
					type = .Line
					return
				}
				type = .Quadratic
				return
			}
			type = .Cusp
			return
		}
		if discr > 0 {
			type = .Serpentine
			return
		}
		type = .Loop
	} else {// float
		cross_1 := [3]T{end.y - c1.y,     c1.x - end.x,     end.x * c1.y - end.y * c1.x}
		cross_2 := [3]T{start.y - end.y,  end.x - start.x,  start.x * end.y - start.y * end.x}
		cross_3 := [3]T{c0.y - start.y,   start.x - c0.x,   c0.x * start.y - c0.y * start.x}

		a1 := start.x * cross_1.x  + start.y * cross_1.y  + cross_1.z
		a2 := c0.x * cross_2.x     + c0.y * cross_2.y     + cross_2.z
		a3 := c1.x * cross_3.x     + c1.y * cross_3.y     + cross_3.z

		d0 := a1 - 2 * a2 + 3 * a3
		d1 := -a2 + 3 * a3
		d2 := 3 * a3

		outD.x = d0
		outD.y = d1
		outD.z = d2

		D     := 3 * d1 * d1 - 4 * d2 * d0
		discr := d0 * d0 * D

		if discr >= -math.epsilon(T) && discr <= math.epsilon(T) {
			if  d0 >= -math.epsilon(T) && d0 <= math.epsilon(T) &&  d1 >= -math.epsilon(T) && d1 <= math.epsilon(T) {
				if d2 >= -math.epsilon(T) && d2 <= math.epsilon(T) {
					type = .Line
					return
				}
				type = .Quadratic
				return
			}
			type = .Cusp
			return
		}
		if discr > math.epsilon(T) {
			type = .Serpentine
			return
		}
		type = .Loop
	}
    return
}

LineSplitCubic :: proc "contextless" (pts:[4][$N]$T, t:T) -> (outPts1:[4][N]T, outPts2:[4][N]T) where intrinsics.type_is_numeric(T) {
    outPts1[0] = pts[0]
    outPts2[3] = pts[3]
    outPts1[1] = linalg.lerp(pts[0], pts[1], t)
    outPts2[2] = linalg.lerp(pts[2], pts[3], t)
    p11 := linalg.lerp(pts[1], pts[2], t)
    outPts1[2] = linalg.lerp(outPts1[1], p11, t)
    outPts2[1] = linalg.lerp(p11, outPts2[2], t)
    outPts1[3] = linalg.lerp(outPts1[2], outPts2[1], t)
    outPts2[0] = outPts1[3]
    return
}

LineSplitQuadratic :: proc "contextless" (pts:[3][$N]$T, t:T) -> (outPts1:[3][N]T, outPts2:[3][N]T) where intrinsics.type_is_numeric(T) {
    outPts1[0] = pts[0]
    outPts2[2] = pts[2]
    outPts1[1] = linalg.lerp(pts[0], pts[1], t)
    outPts2[1] = linalg.lerp(pts[1], pts[2], t)
    outPts1[2] = pts[1]
    outPts2[0] = pts[1]
    return
}

LineSplitLine :: proc "contextless" (pts:[2][$N]$T, t:T) -> (outPts1:[2][N]T, outPts2:[2][N]T) where intrinsics.type_is_numeric(T) {
    outPts1[0] = pts[0]
    outPts1[1] = linalg.lerp(pts[0], pts[1], t)
    outPts2[0] = outPts1[1]
    outPts2[1] = pts[1]
    return
}

@(private="file") _Shapes_ComputeLine :: proc(
    vertList:^[dynamic]shape_vertex2di64,
    indList:^[dynamic]u32,
    outPoly:^[dynamic]CurveStruct,
    color:linalg.point3dw,
    pts:[]linalg.pointi64,
    type:curve_type,
    _subdiv :i64 = 0,
    _repeat :int = -1) -> shape_error {

    assert(_subdiv >= 0)

    curveType := type
    err:shape_error = nil

    reverse := false
    outD:[3]i64 = {0, 0, 0}
    if curveType != .Line && curveType != .Quadratic {
        curveType, err, outD = GetCubicCurveType(pts[0], pts[1], pts[2], pts[3])
        if err != nil do return err
    } else if curveType == .Quadratic && len(pts) == 3 {
		if _subdiv == 0 {
            vlen :u32 = u32(len(vertList))
            if linalg.GetPolygonOrientation(pts) == .CounterClockwise {
                non_zero_append(vertList, shape_vertex2di64{
                    uvw = {0,0,-100},//-100 check this is not cubic curve
                    pos = pts[0],
                    color = color,
                })
                non_zero_append(vertList, shape_vertex2di64{
                    uvw = {-0.5,0,-100},
                    pos = pts[1],
                    color = color,
                })
                non_zero_append(vertList, shape_vertex2di64{
                    uvw = {-1,-1,-100},
                    pos = pts[2],
                    color = color,
                })
            } else {
                non_zero_append(vertList, shape_vertex2di64{
                    uvw = {0,0,-100},
                    pos = pts[0],
                    color = color,
                })
                non_zero_append(vertList, shape_vertex2di64{
                    uvw = {0.5,0,-100},
                    pos = pts[1],
                    color = color,
                })
                non_zero_append(vertList, shape_vertex2di64{
                    uvw = {1,1,-100},
                    pos = pts[2],
                    color = color,
                })
            }
            non_zero_append(indList, vlen, vlen + 1, vlen + 2)
		} else {
			// 반으로 나눈다.
            x01 := (pts[1].x - pts[0].x) * _subdiv / PRECISION + pts[0].x
            y01 := (pts[1].y - pts[0].y) * _subdiv / PRECISION + pts[0].y
            x12 := (pts[2].x - pts[1].x) * _subdiv / PRECISION + pts[1].x
            y12 := (pts[2].y - pts[1].y) * _subdiv / PRECISION + pts[1].y

            x012 := (x12 - x01) * _subdiv / PRECISION + x01
            y012 := (y12 - y01) * _subdiv / PRECISION + y01

            err := _Shapes_ComputeLine(vertList, indList, outPoly, color,{pts[0], { x01, y01 }, { x012, y012 }}, .Quadratic, 0, 0)
            if err != nil do return err
            err = _Shapes_ComputeLine(vertList, indList, outPoly, color,{{ x012, y012 }, { x12, y12 }, pts[2]}, .Quadratic, 0, 0)
            if err != nil do return err
        }
        return nil
    }

    F :matrix[4,3]i64

    reverseOrientation :: #force_inline proc "contextless" (F:matrix[4,3]i64) -> matrix[4,3]i64 {
        return {
            -F[0,0], -F[0,1], F[0,2],
            -F[1,0], -F[1,1], F[1,2],
            -F[2,0], -F[2,1], F[2,2],
            -F[3,0], -F[3,1], F[3,2],
        }
    }
	repeat := 0
    subdiv :i64 = 0

	// digit-by-digit integer sqrt (port of C sqrt_i64) https://github.com/chmike/fpsqrt
	sqrt_i64 :: proc "contextless" (v: i64) -> i64 {
		b := u64(1) << 62
		q: u64 = 0
		r := u64(v)
		for b > r do b >>= 2
		for b > 0 {
			t := q + b
			q >>= 1
			if r >= t {
				r -= t
				q += b
			}
			b >>= 2
		}
		return i64(q)
	}
    if _subdiv == 0.0 {
        switch curveType {
            case .Line:
                return nil
            case .Quadratic:
                F = {
                    0,					0,				0,      
                    PRECISION / 3,		0,				PRECISION / 3,
                    2 * PRECISION / 3,	PRECISION / 3, 	2 * PRECISION / 3,
                    1 * PRECISION,		1 * PRECISION,	1 * PRECISION,
                }
                if outD[2] < 0 do reverse = true
            case .Serpentine:
                t1 := sqrt_i64((9 * outD[1] * outD[1] / PRECISION - 12 * outD[0] * outD[2] / PRECISION) * PRECISION)//mul PRECISION for sqrt
                ls := 3.0 * outD[1] - t1
                lt := 6.0 * outD[0]
                ms := 3.0 * outD[1] + t1
                mt := lt
                ltMinusLs := lt - ls
                mtMinusMs := mt - ms
    
                F = {
                    ls * ms / PRECISION,
					ls * ls / PRECISION * ls / PRECISION,
					ms * ms / PRECISION * ms / PRECISION,

                    (3 * ls * ms / PRECISION - ls * mt / PRECISION - lt * ms / PRECISION) / PRECISION / 3,
					ls * ls / PRECISION * (ls - lt) / PRECISION,
					ms * ms / PRECISION * (ms - mt) / PRECISION,

                    (lt * (mt - 2 * ms) / PRECISION + ls * (3 * ms - 2 * mt) / PRECISION) / PRECISION / 3,
					ltMinusLs * ltMinusLs / PRECISION * ls / PRECISION,
					mtMinusMs * mtMinusMs / PRECISION * ms / PRECISION,

                    ltMinusLs * mtMinusMs / PRECISION,
					-(ltMinusLs * ltMinusLs / PRECISION * ltMinusLs / PRECISION),
					-(mtMinusMs * mtMinusMs / PRECISION * mtMinusMs / PRECISION),
                }
    
                if outD[0] < 0 do reverse = true
            case .Loop:
                t1 := sqrt_i64((4 * outD[0] * outD[2] / PRECISION - 3 * outD[1] * outD[1] / PRECISION) * PRECISION)
                ls := outD[1] - t1
                lt := 2 * outD[0]
                ms := outD[1] + t1
                mt := lt
    
                ql := ls * PRECISION / lt
                qm := ms * PRECISION / mt
               
                if _repeat == -1 && 0 < ql && ql < 1 {
                    repeat = 1
                    subdiv = ql
                } else if _repeat == -1 && 0 < qm && qm < 1 {
                    repeat = 2
                    subdiv = qm
                } else {
                    ltMinusLs := lt - ls
                    mtMinusMs := mt - ms
    
                    F = {
                        ls * ms / PRECISION,
                        ls * ls / PRECISION * ms / PRECISION,
                        ls * ms / PRECISION * ms / PRECISION,

                        (-ls * mt - lt * ms + 3 * ls * ms) / PRECISION / 3,
                        -(ls * (mt - 3 * ms) + 2 * lt * ms) / PRECISION * ls / PRECISION / 3,
                        -(ls * (2 * mt - 3 * ms) + lt * ms) / PRECISION * ms / PRECISION / 3,

                        (lt * (mt - 2.0 * ms) + ls * (3.0 * ms - 2.0 * mt)) / PRECISION / 3,
                        (ls * (2.0 * mt - 3.0 * ms) + lt * ms) / PRECISION / ltMinusLs / PRECISION / 3,
                        (ls * (mt - 3.0 * ms) + 2.0 * lt * ms) / PRECISION / mtMinusMs / PRECISION / 3,

                        ltMinusLs * mtMinusMs / PRECISION, 
						-(ltMinusLs * ltMinusLs) / PRECISION * mtMinusMs / PRECISION,
						-ltMinusLs * mtMinusMs / PRECISION * mtMinusMs / PRECISION,
                    }
                    reverse = (outD[0] > 0 && F[1,0] < 0) || (outD[0] < 0 && F[1,0] > 0)
                }
            case .Cusp:
                ls := outD[2]
                lt := 3 * outD[1]
                lsMinusLt := ls - lt
                F = {
                    ls,                         ls * ls / PRECISION * ls / PRECISION,                       PRECISION,
                    ls - lt / 3,    			ls * ls / PRECISION * lsMinusLt / PRECISION,                PRECISION,
                    ls - 2 * lt / 3,      		lsMinusLt * lsMinusLt / PRECISION * ls / PRECISION,         PRECISION,
                    lsMinusLt,                  lsMinusLt * lsMinusLt / PRECISION * lsMinusLt / PRECISION,  PRECISION,
                }
                //reverse = true
            case .Unknown:
                unreachable()
        }
    }
   
	//반으로 나눈다.
    if repeat > 0 || _subdiv != 0 {
        if subdiv == 0 {
            subdiv = _subdiv
        }
        x01 := (pts[1].x - pts[0].x) * subdiv / PRECISION + pts[0].x
        y01 := (pts[1].y - pts[0].y) * subdiv / PRECISION + pts[0].y
        x12 := (pts[2].x - pts[1].x) * subdiv / PRECISION + pts[1].x
        y12 := (pts[2].y - pts[1].y) * subdiv / PRECISION + pts[1].y

        x23 := (pts[3].x - pts[2].x) * subdiv / PRECISION + pts[2].x
        y23 := (pts[3].y - pts[2].y) * subdiv / PRECISION + pts[2].y

        x012 := (x12 - x01) * subdiv / PRECISION + x01
        y012 := (y12 - y01) * subdiv / PRECISION + y01

        x123 := (x23 - x12) * subdiv / PRECISION + x12
        y123 := (y23 - y12) * subdiv / PRECISION + y12

        x0123 := (x123 - x012) * subdiv / PRECISION + x012
        y0123 := (y123 - y012) * subdiv / PRECISION + y012

        //TODO (xfitgd) 일단은 무조건 곡선을 분할하는 코드를 작성함. 추후에는 필요한 부분만 분할하는 코드로 수정하는 최적화 필요
        if repeat == 2 {//loop에서 분할해야하는경우 일단 2개로만 분할한다.
            err := _Shapes_ComputeLine(vertList, indList, outPoly, color,{pts[0], { x01, y01 }, { x012, y012 }, { x0123, y0123 }}, type, 0, 1)
            if err != nil do return err
            err = _Shapes_ComputeLine(vertList, indList, outPoly, color,{{ x0123, y0123 }, { x123, y123 }, { x23, y23 }, pts[3]}, type, 0, 0)
            if err != nil do return err
        } else if repeat == 1 {
            err := _Shapes_ComputeLine(vertList, indList, outPoly, color,{pts[0], { x01, y01 }, { x012, y012 }, { x0123, y0123 }}, type, 0, 0)
            if err != nil do return err
            err = _Shapes_ComputeLine(vertList, indList, outPoly, color,{{ x0123, y0123 }, { x123, y123 }, { x23, y23 }, pts[3]}, type, 0, 1)
            if err != nil do return err
        } else {
            if _repeat == 3 {//4개로 분할한다.
                err := _Shapes_ComputeLine(vertList, indList, outPoly, color,{pts[0], { x01, y01 }, { x012, y012 }, { x0123, y0123 }}, type, 0, 0)
                if err != nil do return err
                err = _Shapes_ComputeLine(vertList, indList, outPoly, color,{{ x0123, y0123 }, { x123, y123 }, { x23, y23 }, pts[3]}, type, 0, 0)
                if err != nil do return err
            } else {
                err := _Shapes_ComputeLine(vertList, indList, outPoly, color,{pts[0], { x01, y01 }, { x012, y012 }, { x0123, y0123 }}, type, PRECISION / 2, 3)
                if err != nil do return err
                err = _Shapes_ComputeLine(vertList, indList, outPoly, color,{{ x0123, y0123 }, { x123, y123 }, { x23, y23 }, pts[3]}, type, PRECISION / 2, 3)
                if err != nil do return err
            }
        }
        return nil
    }
    if repeat == 1 {
        reverse = !reverse
    }

    if reverse {
      F = reverseOrientation(F)
    }

    appendLine :: proc (vertList:^[dynamic]shape_vertex2di64, indList:^[dynamic]u32, color:linalg.point3dw, pts:[]linalg.pointi64, F:matrix[4,3]i64) {
        if len(pts) == 2 {
            return
        }
        start :u32 = u32(len(vertList))
        non_zero_append(vertList, shape_vertex2di64{
            uvw = {f32(f64(F[0,0])/PRECISIONF), f32(f64(F[0,1])/PRECISIONF), f32(f64(F[0,2])/PRECISIONF)},
            color = color,
        })
        non_zero_append(vertList, shape_vertex2di64{
            uvw = {f32(f64(F[1,0])/PRECISIONF), f32(f64(F[1,1])/PRECISIONF), f32(f64(F[1,2])/PRECISIONF)},
            color = color,
        })
        non_zero_append(vertList, shape_vertex2di64{
            uvw = {f32(f64(F[2,0])/PRECISIONF), f32(f64(F[2,1])/PRECISIONF), f32(f64(F[2,2])/PRECISIONF)},
            color = color,
        })
        non_zero_append(vertList, shape_vertex2di64{
            uvw = {f32(f64(F[3,0])/PRECISIONF), f32(f64(F[3,1])/PRECISIONF), f32(f64(F[3,2])/PRECISIONF)},
            color = color,
        })
        vertList[start].pos = pts[0]
        vertList[start+1].pos = pts[1]
        vertList[start+2].pos = pts[2]
        vertList[start+3].pos = pts[3]
        //triangulate
        for i:u32 = 0; i < 4; i += 1 {
            for j:u32 = i + 1; j < 4; j += 1 {
                if vertList[start + i].pos == vertList[start + j].pos {
                    indices :[3]u32 = {start, start, start}
                    idx:u32 = 0
                    for k:u32 = 0; k < 4; k += 1 {
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
        for i:u32 = 0; i < 4; i += 1 {
            indices :[3]u32 = {start, start, start}
            idx:u32 = 0
            for j:u32 = 0; j < 4; j += 1 {
                if j != i {
                    indices[idx] += j
                    idx += 1
                }
            }
            if linalg.PointInTriangle_Int(vertList[start + i].pos, vertList[indices[0]].pos, vertList[indices[1]].pos, vertList[indices[2]].pos, PRECISION) {
                for k:u32 = 0; k < 3; k += 1 {
                    non_zero_append(indList, indices[k])
                    non_zero_append(indList, indices[(k + 1)%3])
                    non_zero_append(indList, start + i)
                }
                return
            }
        }

        b := linalg.LinesIntersect(vertList[start].pos, vertList[start + 2].pos, vertList[start + 1].pos, vertList[start + 3].pos)
        if b {
            if linalg.length2(vertList[start + 2].pos - vertList[start].pos) < linalg.length2(vertList[start + 3].pos - vertList[start + 1].pos) {
                non_zero_append(indList, start, start + 1, start + 2, start, start + 2, start + 3)
            } else {
                non_zero_append(indList, start, start + 1, start + 3, start + 1, start + 2, start + 3)
            }
            return
        }
        b = linalg.LinesIntersect(vertList[start].pos, vertList[start + 3].pos, vertList[start + 1].pos, vertList[start + 2].pos)
        if b {
            if linalg.length2(vertList[start + 3].pos - vertList[start].pos) < linalg.length2(vertList[start + 2].pos - vertList[start + 1].pos) {
                non_zero_append(indList, start, start + 1, start + 3, start, start + 3, start + 2)
            } else {
                non_zero_append(indList, start, start + 1, start + 2, start + 2, start + 1, start + 3)
            }
            return
        }
        if linalg.length2(vertList[start + 1].pos - vertList[start].pos) < linalg.length2(vertList[start + 3].pos - vertList[start + 2].pos) {
            non_zero_append(indList, start, start + 2, start + 1, start, start + 1, start + 3)
        } else {
            non_zero_append(indList, start, start + 2, start + 3, start + 3, start + 2, start + 1)
        }
    }
    appendLine(vertList, indList, color, pts[:len(pts)], F)

    if len(pts) == 3 {
        non_zero_append(outPoly, CurveStruct{pts[0], false}, CurveStruct{pts[1], true})
    } else {
        non_zero_append(outPoly, CurveStruct{pts[0], false}, CurveStruct{pts[1], true}, CurveStruct{pts[2], true})
    }

    return nil
}

cvt_raw_shapei64_to_raw_shape :: proc(raw64:raw_shapei64, allocator := context.allocator) ->  (res:raw_shape, err:runtime.Allocator_Error) #optional_allocator_error {
	res.vertices = mem.make_non_zeroed([]shape_vertex2d, len(raw64.vertices), allocator) or_return
	defer if err != nil do delete(res.vertices, allocator)
	res.indices = mem.make_non_zeroed([]u32, len(raw64.indices), allocator) or_return
	defer if err != nil do delete(res.indices, allocator)// not working

	for v, i in raw64.vertices {
		res.vertices[i].pos.x = f32(fixed.to_f64(v.pos.x))
		res.vertices[i].pos.y = f32(fixed.to_f64(v.pos.y))

		res.vertices[i].color = v.color
		res.vertices[i].uvw = v.uvw
	}
	mem.copy_non_overlapping(raw_data(raw64.indices), raw_data(raw64.indices), len(raw64.indices) * size_of(u32))

	return
}

cvt_raw_shape_to_raw_shapei64 :: proc(raw:raw_shape, allocator := context.allocator) ->  (res:raw_shapei64, err:runtime.Allocator_Error) #optional_allocator_error {
	res.vertices = mem.make_non_zeroed([]shape_vertex2di64, len(raw.vertices), allocator) or_return
	defer if err != nil do delete(res.vertices, allocator)
	res.indices = mem.make_non_zeroed([]u32, len(raw.indices), allocator) or_return
	defer if err != nil do delete(res.indices, allocator)// not working

	for v, i in raw.vertices {
        fixed.init_from_f64(&res.vertices[i].pos.x, f64(v.pos.x))
        fixed.init_from_f64(&res.vertices[i].pos.y, f64(v.pos.y))

		res.vertices[i].color = v.color
		res.vertices[i].uvw = v.uvw
	}
	mem.copy_non_overlapping(raw_data(res.indices), raw_data(raw.indices), len(res.indices) * size_of(u32))

	return
}

cvt_shapes_to_shapesi64 :: proc(poly:shapes, allocator := context.allocator) ->  (poly64:shapesi64, err:runtime.Allocator_Error) #optional_allocator_error {
	poly64 = shapesi64{
		nodes = mem.make_non_zeroed([]shape_nodei64, len(poly.nodes), allocator) or_return
	}

	for n, i in poly.nodes {
		poly64.nodes[i].pts = mem.make_non_zeroed([][2]FixedDef, len(n.pts), allocator)
		for p, j in n.pts {
            fixed.init_from_f64(&poly64.nodes[i].pts[j].x, f64(p.x))
            fixed.init_from_f64(&poly64.nodes[i].pts[j].y, f64(p.y))
		}

		if n.curve_pts_ids != nil {
			poly64.nodes[i].curve_pts_ids = mem.make_non_zeroed([]u32, len(n.curve_pts_ids), allocator)
			mem.copy_non_overlapping(raw_data(poly64.nodes[i].curve_pts_ids), raw_data(n.curve_pts_ids), len(n.curve_pts_ids) * size_of(u32))
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

cvt_shapesi64_to_shapes :: proc(poly:shapesi64, allocator := context.allocator) -> (poly32:shapes, err:runtime.Allocator_Error) #optional_allocator_error {
	poly32 = shapes{
		nodes = mem.make_non_zeroed([]shape_node, len(poly.nodes), allocator) or_return
	}

	for n, i in poly.nodes {
		poly32.nodes[i].pts = mem.make_non_zeroed([]linalg.point, len(n.pts), allocator)
		for p, j in n.pts {
			poly32.nodes[i].pts[j].x = f32(fixed.to_f64(p.x))
			poly32.nodes[i].pts[j].y = f32(fixed.to_f64(p.y))
		}

		if n.curve_pts_ids != nil {
			poly32.nodes[i].curve_pts_ids = mem.make_non_zeroed([]u32, len(n.curve_pts_ids), allocator)
			mem.copy_non_overlapping(raw_data(poly32.nodes[i].curve_pts_ids), raw_data(n.curve_pts_ids), len(n.curve_pts_ids) * size_of(u32))
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

shapes_compute_polygon :: proc(poly:shapes, allocator := context.allocator) -> (res:raw_shape, err:shape_error = nil) {
	poly64 := shapesi64{
		nodes = mem.make_non_zeroed([]shape_nodei64, len(poly.nodes), context.temp_allocator) or_return
	}

	for n, i in poly.nodes {
		poly64.nodes[i].pts = mem.make_non_zeroed([]FixedDef, len(n.pts), context.temp_allocator)
		for p, j in n.pts {
			poly64.nodes[i].pts[j].x = i64(f64(p.x) * PRECISIONF)
			poly64.nodes[i].pts[j].y = i64(f64(p.y) * PRECISIONF)
		}

		if n.curve_pts_ids != nil {
			poly64.nodes[i].curve_pts_ids = mem.make_non_zeroed([]u32, len(n.curve_pts_ids), context.temp_allocator)
			mem.copy_non_overlapping(raw_data(poly64.nodes[i].curve_pts_ids), raw_data(n.curve_pts_ids), len(n.curve_pts_ids) * size_of(u32))
		} else {
			poly64.nodes[i].curve_pts_ids = nil
		}

		poly64.nodes[i].is_closed = n.is_closed
		poly64.nodes[i].thickness = i64(f64(n.thickness) * PRECISIONF)
		poly64.nodes[i].color = n.color
		poly64.nodes[i].stroke_color = n.stroke_color
	}

	res64 := shapes_compute_polygoni64(poly64, context.temp_allocator) or_return

	res.vertices = mem.make_non_zeroed([]shape_vertex2d, len(res64.vertices), allocator) or_return
	defer if err != nil do delete(res.vertices, allocator)
	res.indices = mem.make_non_zeroed([]u32, len(res64.indices), allocator) or_return
	defer if err != nil do delete(res.indices, allocator)// not working

	for v, i in res64.vertices {
		res.vertices[i].pos.x = f32(f64(v.pos.x) / PRECISIONF)
		res.vertices[i].pos.y = f32(f64(v.pos.y) / PRECISIONF)

		res.vertices[i].color = v.color
		res.vertices[i].uvw = v.uvw
	}

	mem.copy_non_overlapping(raw_data(res.indices), raw_data(res64.indices), len(res64.indices) * size_of(u32))
	return
}

@private CURVE_STRUCT :: struct {
	start:linalg.pointi64,
	ctl0:linalg.pointi64,
	ctl1:linalg.pointi64,
	end:linalg.pointi64,
	type:curve_type,
}

shapes_compute_polygoni64 :: proc(poly:shapesi64, allocator := context.allocator) -> (res:raw_shapei64, err:shape_error = nil) {
    vertList:[dynamic]shape_vertex2di64 = make([dynamic]shape_vertex2di64, context.temp_allocator)
    indList:[dynamic]u32 = make([dynamic]u32, context.temp_allocator)

    shapes_compute_polygon_in :: proc(vertList:^[dynamic]shape_vertex2di64, indList:^[dynamic]u32, poly:shapesi64, allocator : runtime.Allocator) -> (err:shape_error = nil) {	
        for node, nidx in poly.nodes {
			non_curves:[dynamic]linalg.pointi64 = make([dynamic]linalg.pointi64, context.temp_allocator)
			curves:[dynamic]CURVE_STRUCT = make([dynamic]CURVE_STRUCT, context.temp_allocator)

			if node.color.a > 0 {
				curve_idx := 0
				for pt, i in node.pts {
					//TODO
					// pts : [4]linalg.pointi64
					// _Shapes_ComputeLine(
					// 	vertList,
					// 	indList,
					// 	&outPoly[poly_idx],
					// 	node.color,
					// 	pts[:3],
					// 	.Quadratic, PRECISION / 2) or_return
					// if err != nil do return

					// _Shapes_ComputeLine(
					// 	vertList,
					// 	indList,
					// 	&outPoly[poly_idx],
					// 	node.color,
					// 	pts[:4],
					// 	.Unknown, PRECISION / 2) or_return//TODO (xfitgd) 일단은 0.5로 고정

					if curve_idx < len(node.curve_pts_ids) && node.curve_pts_ids[curve_idx] == u32(i) {//곡선 점이면
						if curve_idx + 1 < len(node.curve_pts_ids) && node.curve_pts_ids[curve_idx + 1] == u32(i) {//큐빅인지 확인
							non_zero_append(&curves, CURVE_STRUCT{
								start = node.pts[i - 1],
								ctl0 = node.pts[i],
								ctl1 = node.pts[i + 1],
								end = node.pts[i + 2 >= len(node.pts) ? 0 : i + 2],
                                type = .Unknown,
							})
							curve_idx += 2
						} else {
                            non_zero_append(&curves, CURVE_STRUCT{
								start = node.pts[i - 1],
								ctl0 = node.pts[i],
								end = node.pts[i + 1 >= len(node.pts) ? 0 : i + 1],
                                type = .Quadratic,
							})
							curve_idx += 1
						}
					} else {
						non_zero_append(&non_curves, pt)
					}
				}
			}
		}
		return
	}
	return
}

poly_transform_matrix :: proc "contextless" (inout_poly: ^shapes, F: linalg.matrix44) {
	for &node in inout_poly.nodes {
		for &pts in node.pts {
			out := linalg.mul(F, linalg.point3dw{pts.x, pts.y, 0, 1})
			pts = linalg.point{out.x, out.y} / out.w
		}
	}
}