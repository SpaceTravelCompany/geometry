#+feature using-stmt
package geometry

import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:math/linalg"
import "core:mem"

import "core:math/fixed"


//Cover MAX 741455 * 741455
FIXED_SHIFT :: 24
FixedDef :: fixed.Fixed(i64, FIXED_SHIFT)
@private Fixed1 :: fixed.Fixed(i64, FIXED_SHIFT){i = 1 << FIXED_SHIFT}
@private Fixed2 :: fixed.Fixed(i64, FIXED_SHIFT){i = 2 << FIXED_SHIFT}
@private Fixed3 :: fixed.Fixed(i64, FIXED_SHIFT){i = 3 << FIXED_SHIFT}
@private Div2Fixed :: fixed.Fixed(i64, FIXED_SHIFT){i = (1 << FIXED_SHIFT) / 2}
@private Div3Fixed :: fixed.Fixed(i64, FIXED_SHIFT){i = (1 << FIXED_SHIFT) / 3}
@private Div3Mul2Fixed :: fixed.Fixed(i64, FIXED_SHIFT){i = (2 << FIXED_SHIFT) / 3}


shape_vertex2di64 :: struct #align(1) {
    pos: [2]FixedDef,
    uvw: linalg.Vector3f32,
    color: linalg.Vector4f32,
};

shape_vertex2d :: struct #align(1) {
    pos: linalg.Vector2f32,
    uvw: linalg.Vector3f32,
    color: linalg.Vector4f32,
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
    pts: []linalg.Vector2f32,
	curve_pts_ids: []u32,
    color: linalg.Vector4f32,
    stroke_color: linalg.Vector4f32,
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
    color: linalg.Vector4f32,
    stroke_color: linalg.Vector4f32,
    thickness: FixedDef,
	is_closed: bool,
}

shapesi64 :: struct {
    nodes: []shape_nodei64,
}

CvtQuadraticToCubic0 :: #force_inline proc "contextless" (_start : linalg.Vector2f32, _control : linalg.Vector2f32) -> linalg.Vector2f32 {
    return linalg.Vector2f32{ _start.x + (2.0/3.0) * (_control.x - _start.x), _start.y + (2.0/3.0) * (_control.y - _start.y) }
}
CvtQuadraticToCubic1 :: #force_inline proc "contextless" (_end : linalg.Vector2f32, _control : linalg.Vector2f32) -> linalg.Vector2f32 {
    return CvtQuadraticToCubic0(_end, _control)
}

rect_line_init :: proc "contextless" (_rect: Rectf32) -> [4]linalg.Vector2f32 {
	return [4]linalg.Vector2f32{linalg.Vector2f32{_rect.left, _rect.top}, linalg.Vector2f32{_rect.left, _rect.bottom}, linalg.Vector2f32{_rect.right, _rect.bottom}, linalg.Vector2f32{_rect.right, _rect.top}}
}

round_rect_line_init :: proc "contextless" (_rect: Rectf32, _radius: f32) -> (pts:[16]linalg.Vector2f32, curve_pts_ids:[8]u32) {
	r := _radius
	// Clamp radius to fit within rect
	half_width := (_rect.right - _rect.left) * 0.5
	half_height := abs(_rect.bottom - _rect.top) * 0.5
	r = min(r, min(half_width, half_height))
	
	t: f32 = (4.0 / 3.0) * math.tan_f32(math.PI / 8.0)
	tt := t * r
	
	// Corner centers
	top_left := linalg.Vector2f32{_rect.left + r, _rect.top + r}
	top_right := linalg.Vector2f32{_rect.right - r, _rect.top + r}
	bottom_right := linalg.Vector2f32{_rect.right - r, _rect.bottom - r}
	bottom_left := linalg.Vector2f32{_rect.left + r, _rect.bottom - r}
	
	return [16]linalg.Vector2f32{
		// Top-left corner (cubic) - counter-clockwise: from top to left
		// Note: y increases upward, _rect.top is top (larger y), _rect.bottom is bottom (smaller y)
			linalg.Vector2f32{_rect.left + r, _rect.top},
			linalg.Vector2f32{_rect.left + r - tt, _rect.top},
			linalg.Vector2f32{_rect.left, _rect.top - r + tt},
		// Left line - counter-clockwise: from top to bottom (y decreases)
			linalg.Vector2f32{_rect.left, _rect.top - r},
		// Bottom-left corner (cubic) - counter-clockwise: from left to bottom
			linalg.Vector2f32{_rect.left, _rect.bottom + r},
			linalg.Vector2f32{_rect.left, _rect.bottom + r - tt},
			linalg.Vector2f32{_rect.left + r - tt, _rect.bottom},
		// Bottom line - counter-clockwise: from left to right
			linalg.Vector2f32{_rect.left + r, _rect.bottom},
		// Bottom-right corner (cubic) - counter-clockwise: from bottom to right
			linalg.Vector2f32{_rect.right - r, _rect.bottom},
			linalg.Vector2f32{_rect.right - r + tt, _rect.bottom},
			linalg.Vector2f32{_rect.right, _rect.bottom + r - tt},
		// Right line - counter-clockwise: from bottom to top (y increases)
			linalg.Vector2f32{_rect.right, _rect.bottom + r},
		// Top-right corner (cubic) - counter-clockwise: from right to top
			linalg.Vector2f32{_rect.right, _rect.top - r},
			linalg.Vector2f32{_rect.right, _rect.top - r + tt},
			linalg.Vector2f32{_rect.right - r + tt, _rect.top},
		// Top line - counter-clockwise: from right to left
			linalg.Vector2f32{_rect.right - r, _rect.top},
	}, [8]u32{1,2, 5,6, 9,10, 13,14}
}

circle_cubic_init :: proc "contextless" (_center: linalg.Vector2f32, _r: f32) -> (pts:[12]linalg.Vector2f32, curve_pts_ids:[8]u32) {
	t: f32 = (4.0 / 3.0) * math.tan_f32(math.PI / 8.0)
	tt := t * _r
	return [12]linalg.Vector2f32{
			linalg.Vector2f32{_center.x - _r, _center.y},
			linalg.Vector2f32{_center.x - _r, _center.y - tt},
			linalg.Vector2f32{_center.x - tt, _center.y - _r},

			linalg.Vector2f32{_center.x, _center.y - _r},
			linalg.Vector2f32{_center.x + tt, _center.y - _r},
			linalg.Vector2f32{_center.x + _r, _center.y - tt},

			linalg.Vector2f32{_center.x + _r, _center.y},
			linalg.Vector2f32{_center.x + _r, _center.y + tt},
			linalg.Vector2f32{_center.x + tt, _center.y + _r},

			linalg.Vector2f32{_center.x, _center.y + _r},
			linalg.Vector2f32{_center.x - tt, _center.y + _r},
			linalg.Vector2f32{_center.x - _r, _center.y + tt},
	}, [8]u32{1,2, 4,5, 7,8, 10,11}
}

ellipse_cubic_init :: proc "contextless" (_center: linalg.Vector2f32, _rxy: linalg.Vector2f32) -> (pts:[12]linalg.Vector2f32, curve_pts_ids:[8]u32) {
	t: f32 = (4.0 / 3.0) * math.tan_f32(math.PI / 8.0)
	ttx := t * _rxy.x
	tty := t * _rxy.y
	return [12]linalg.Vector2f32{
			linalg.Vector2f32{_center.x - _rxy.x, _center.y},
			linalg.Vector2f32{_center.x - _rxy.x, _center.y - tty},
			linalg.Vector2f32{_center.x - ttx, _center.y - _rxy.y},

			linalg.Vector2f32{_center.x, _center.y - _rxy.y},
			linalg.Vector2f32{_center.x + ttx, _center.y - _rxy.y},
			linalg.Vector2f32{_center.x + _rxy.x, _center.y - tty},

			linalg.Vector2f32{_center.x + _rxy.x, _center.y},
			linalg.Vector2f32{_center.x + _rxy.x, _center.y + tty},
			linalg.Vector2f32{_center.x + ttx, _center.y + _rxy.y},

			linalg.Vector2f32{_center.x, _center.y + _rxy.y},
			linalg.Vector2f32{_center.x - ttx, _center.y + _rxy.y},
			linalg.Vector2f32{_center.x - _rxy.x, _center.y + tty},
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
    res.vertices = make_non_zeroed_slice([]shape_vertex2di64, len(self.vertices), allocator) or_return
    defer if err != nil do delete(res.vertices, allocator)

    res.indices = make_non_zeroed_slice([]u32, len(self.indices), allocator) or_return

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

    res.vertices = make_non_zeroed_slice([]shape_vertex2d, len(self.vertices), allocator) or_return
    defer if err != nil do delete(res.vertices, allocator)

    res.indices = make_non_zeroed_slice([]u32, len(self.indices), allocator) or_return

    intrinsics.mem_copy_non_overlapping(&res.vertices[0], &self.vertices[0], len(self.vertices) * size_of(shape_vertex2d))
    intrinsics.mem_copy_non_overlapping(&res.indices[0], &self.indices[0], len(self.indices) * size_of(u32))
    return
}

GetCubicCurveType :: proc "contextless" (start:[2]$T, control0:[2]T, control1:[2]T, end:[2]T) ->
(type:curve_type = .Unknown, d0:T, d1:T, d2:T, err:shape_error = nil) where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
    if start == control0 && control0 == control1 && control1 == end {
        err = .IsPointNotLine
        return
    }

	when intrinsics.type_is_specialization_of(T, fixed.Fixed) {
        using fixed
		cross_1 := [3]T{sub(end.y, control1.y),     sub(control1.x, end.x),     sub(mul(end.x, control1.y), mul(end.y, control1.x))}
		cross_2 := [3]T{sub(start.y, end.y),  sub(end.x, start.x),  sub(mul(start.x, end.y), mul(start.y, end.x))}
		cross_3 := [3]T{sub(control0.y, start.y),   sub(start.x, control0.x),   sub(mul(control0.x, start.y), mul(control0.y, start.x))}

		a1 := add(add(mul(start.x, cross_1.x), mul(start.y, cross_1.y)), cross_1.z)
		a2 := add(add(mul(control0.x, cross_2.x), mul(control0.y, cross_2.y)), cross_2.z)
		a3 := add(add(mul(control1.x, cross_3.x), mul(control1.y, cross_3.y)), cross_3.z)

		d0 = T{i=a1.i - 2 * a2.i + 3 * a3.i}
		d1 = T{i=-a2.i + 3 * a3.i}
		d2 = T{i=3 * a3.i}

		D     := T{i=3 * mul(d1, d1).i - 4 * mul(d2, d0).i}
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
	} else {// float
		cross_1 := [3]T{end.y - c1.y,     c1.x - end.x,     end.x * c1.y - end.y * c1.x}
		cross_2 := [3]T{start.y - end.y,  end.x - start.x,  start.x * end.y - start.y * end.x}
		cross_3 := [3]T{control0.y - start.y,   start.x - control0.x,   control0.x * start.y - control0.y * start.x}

		a1 := start.x * cross_1.x  + start.y * cross_1.y  + cross_1.z//9
		a2 := control0.x * cross_2.x     + control0.y * cross_2.y     + cross_2.z//7
		a3 := control1.x * cross_3.x     + control1.y * cross_3.y     + cross_3.z//7

		d0 = a1 - 2 * a2 + 3 * a3//27
		d1 = -a2 + 3 * a3//16
		d2 = 3 * a3//8

		D     := 3 * d1 * d1 - 4 * d2 * d0//33 + 36 + 1 = 70
		discr := d0 * d0 * D//27 + 27 + 70 = 124

        EP :: math.epsilon(T) * 250// about count float operations
		if discr >= -EP && discr <= EP {
			if  d0 >= -EP && d0 <= EP &&  d1 >= -EP && d1 <= EP {
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
	}
    return
}

@(private="file") _Shapes_ComputeLine :: proc(
    vertList:^[dynamic]shape_vertex2di64,
    indList:^[dynamic]u32,
    color:linalg.Vector4f32,
    pts:[][2]FixedDef,
    type:curve_type,
    _subdiv :FixedDef = {},
    _repeat :int = -1) -> shape_error {

    using fixed
    assert(_subdiv.i >= 0)

    curveType := type
    err:shape_error = nil

    reverse := false
    d0,d1,d2:FixedDef
    if curveType != .Line && curveType != .Quadratic {
        curveType, d0,d1,d2, err = GetCubicCurveType(pts[0], pts[1], pts[2], pts[3])
        if err != nil do return err
    } else if curveType == .Quadratic && len(pts) == 3 {
		if _subdiv.i == 0 {
            vlen :u32 = u32(len(vertList))
            if GetPolygonOrientation(pts) == .CounterClockwise {
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
            pt01 := [2]FixedDef{
                add(pts[0].x, mul(sub(pts[1].x, pts[0].x), _subdiv)),
                add(pts[0].y, mul(sub(pts[1].y, pts[0].y), _subdiv)),
            }
            pt12 := [2]FixedDef{
                add(pts[1].x, mul(sub(pts[2].x, pts[1].x), _subdiv)),
                add(pts[1].y, mul(sub(pts[2].y, pts[1].y), _subdiv)),
            }
            pt012 := [2]FixedDef{
                add(pt01.x, mul(sub(pt12.x, pt01.x), _subdiv)),
                add(pt01.y, mul(sub(pt12.y, pt01.y), _subdiv)),
            }

            subdivZero := FixedDef{}
            err := _Shapes_ComputeLine(vertList, indList, color, {pts[0], pt01, pt012}, .Quadratic, subdivZero, 0)
            if err != nil do return err
            err = _Shapes_ComputeLine(vertList, indList, color, {pt012, pt12, pts[2]}, .Quadratic, subdivZero, 0)
            if err != nil do return err
        }
        return nil
    }

    F :[4][3]FixedDef

    reverseOrientation :: #force_inline proc "contextless" (F:[4][3]FixedDef) -> [4][3]FixedDef {
        return [4][3]FixedDef{
            {sign(F[0][0]), sign(F[0][1]), F[0][2]},
            {sign(F[1][0]), sign(F[1][1]), F[1][2]},
            {sign(F[2][0]), sign(F[2][1]), F[2][2]},
            {sign(F[3][0]), sign(F[3][1]), F[3][2]},
        }
    }
	repeat := 0
    subdiv :FixedDef = {}

    if _subdiv.i == 0 {
        switch curveType {
            case .Line:
                return nil
            case .Quadratic:
                F = {{{}, {}, {}},	    
                    {Div3Fixed, {},	Div3Fixed},
                    {Div3Mul2Fixed,	Div3Fixed, 	Div3Mul2Fixed},
                    {Fixed1, Fixed1, Fixed1},
                }
                if d2.i < 0 do reverse = true
            case .Serpentine:
                t1 := FixedDef{i=sqrt_i64((9 * mul(d1, d1).i - 12 * mul(d0, d2).i) << FIXED_SHIFT)}//mul PRECISION for sqrt
                ls := FixedDef{i=3 * d1.i - t1.i}
                lt := FixedDef{i=6 * d0.i}
                ms := FixedDef{i=3 * d1.i + t1.i}
                mt := lt
                ltMinusLs := sub(lt, ls)
                mtMinusMs := sub(mt, ms)

                lsls := mul(ls, ls)
                msms := mul(ms, ms)
    
                F = {{mul(ls, ms),
					mul(lsls, ls),
					mul(msms, ms)},

                    {FixedDef{i=3 * mul(ls, ms).i - mul(ls, mt).i - mul(lt, ms).i / 3},
					mul(lsls, sub(ls, lt)),
					mul(lsls, sub(ms, mt))},

                    {FixedDef{i=(mul(lt, FixedDef{i=(mt.i - 2 * ms.i)}).i + mul(ls, FixedDef{i=(3 * ms.i - 2 * mt.i)}).i) / 3},
					mul(mul(ltMinusLs, ltMinusLs), ls),
					mul(mul(mtMinusMs, mtMinusMs), ms)},

                    {mul(ltMinusLs, mtMinusMs),
					sign(mul(mul(ltMinusLs, ltMinusLs), ltMinusLs)),
					sign(mul(mul(mtMinusMs, mtMinusMs), mtMinusMs))},
                }
    
                if d0.i < 0 do reverse = true
            case .Loop:
                t1 := FixedDef{i=sqrt_i64((4 * d0.i * d2.i - 3 * d1.i * d1.i) << FIXED_SHIFT)}
                ls := sub(d1, t1)
                lt := mul(Fixed2, d0)
                ms := add(d1, t1)
                mt := lt
    
                ql := div(ls, lt)
                qm := div(ms, mt)
               
                if _repeat == -1 && 0 < ql.i && ql.i < 1 {
                    repeat = 1
                    subdiv = ql
                } else if _repeat == -1 && 0 < qm.i && qm.i < 1 {
                    repeat = 2
                    subdiv = qm
                } else {
                    ltMinusLs := sub(lt, ls)
                    mtMinusMs := sub(mt, ms)
    
                    lsms := mul(ls, ms)
                    ltms := mul(lt, ms)
                    F = {
                        {lsms,
                        mul(ls, lsms),
                        mul(ms, lsms)},

                        {div( add( sub(mul(sign(ls), mt), ltms), mul(Fixed3, lsms) ), Fixed3),//(-ls * mt - ltms + 3 * lsms) / 3
                        div(mul(sign(add(mul(ls, (sub(mt, mul(Fixed3, ms)))), mul(Fixed2, ltms))), ls), Fixed3),//-(ls * (mt - 3 * ms) + 2 * ltms) * ls / 3
                        div(mul(sign(add(mul(ls, (sub(mul(Fixed2, mt), mul(Fixed3, ms)))), ltms)), ms), Fixed3)},//-(ls * (2 * mt - 3 * ms) + ltms) * ms / 3

                        {div(add(mul(lt, sub(mt, mul(Fixed2, ms))), mul(ls, sub(mul(Fixed3, ms), mul(Fixed2, mt)))), Fixed3),//(lt * (mt - 2.0 * ms) + ls * (3.0 * ms - 2.0 * mt)) / 3
                        div(div(add(mul(ls, sub(mul(Fixed2, mt), mul(Fixed3, ms))), ltms), ltMinusLs), Fixed3),//(ls * (2.0 * mt - 3.0 * ms) + ltms) / ltMinusLs / 3
                        div(div(add(mul(ls, sub(mt, mul(Fixed3, ms))), mul(Fixed2, ltms)), mtMinusMs), Fixed3)},//(ls * (mt - 3.0 * ms) + 2.0 * ltms) / mtMinusMs / 3

                        {mul(ltMinusLs, mtMinusMs),
						sign(mul(mul(ltMinusLs, ltMinusLs), mtMinusMs)),
						sign(mul(ltMinusLs, mul(mtMinusMs, mtMinusMs)))},
                    }
                    reverse = (d0.i > 0 && F[1][0].i < 0) || (d0.i < 0 && F[1][0].i > 0)
                }
            case .Cusp:
                ls := d2
                lt := mul(Fixed3, d1)
                lsMinusLt := sub(ls, lt)
                lsls := mul(ls, ls)
                lsMinusLtSq := mul(lsMinusLt, lsMinusLt)
                F = {
                    {ls,
                    div(mul(lsls, ls), mul(Fixed1, Fixed1)),
                    Fixed1},

                    {sub(ls, div(lt, Fixed3)),
                    div(mul(lsls, lsMinusLt), mul(Fixed1, Fixed1)),
                    Fixed1},

                    {sub(ls, div(mul(Fixed2, lt), Fixed3)),
                    div(mul(lsMinusLtSq, ls), mul(Fixed1, Fixed1)),
                    Fixed1},

                    {lsMinusLt,
                    div(mul(lsMinusLtSq, lsMinusLt), mul(Fixed1, Fixed1)),
                    Fixed1},
                }
                //reverse = true
            case .Unknown:
                unreachable()
        }
    }
   
	//반으로 나눈다.
    if repeat > 0 || _subdiv.i != 0 {
        if subdiv.i == 0 {
            subdiv = _subdiv
        }
        pt01 := [2]FixedDef{
            add(pts[0].x, mul(sub(pts[1].x, pts[0].x), subdiv)),
            add(pts[0].y, mul(sub(pts[1].y, pts[0].y), subdiv)),
        }
        pt12 := [2]FixedDef{
            add(pts[1].x, mul(sub(pts[2].x, pts[1].x), subdiv)),
            add(pts[1].y, mul(sub(pts[2].y, pts[1].y), subdiv)),
        }
        pt23 := [2]FixedDef{
            add(pts[2].x, mul(sub(pts[3].x, pts[2].x), subdiv)),
            add(pts[2].y, mul(sub(pts[3].y, pts[2].y), subdiv)),
        }
        pt012 := [2]FixedDef{
            add(pt01.x, mul(sub(pt12.x, pt01.x), subdiv)),
            add(pt01.y, mul(sub(pt12.y, pt01.y), subdiv)),
        }
        pt123 := [2]FixedDef{
            add(pt12.x, mul(sub(pt23.x, pt12.x), subdiv)),
            add(pt12.y, mul(sub(pt23.y, pt12.y), subdiv)),
        }
        pt0123 := [2]FixedDef{
            add(pt012.x, mul(sub(pt123.x, pt012.x), subdiv)),
            add(pt012.y, mul(sub(pt123.y, pt012.y), subdiv)),
        }

        if repeat == 2 {
            err := _Shapes_ComputeLine(vertList, indList, color, {pts[0], pt01, pt012, pt0123}, type, {}, 1)
            if err != nil do return err
            err = _Shapes_ComputeLine(vertList, indList, color, {pt0123, pt123, pt23, pts[3]}, type, {}, 0)
            if err != nil do return err
        } else if repeat == 1 {
            err := _Shapes_ComputeLine(vertList, indList, color, {pts[0], pt01, pt012, pt0123}, type, {}, 0)
            if err != nil do return err
            err = _Shapes_ComputeLine(vertList, indList, color, {pt0123, pt123, pt23, pts[3]}, type, {}, 1)
            if err != nil do return err
        }
        return nil
    }
    if repeat == 1 {
        reverse = !reverse
    }

    if reverse {
      F = reverseOrientation(F)
    }

    appendLine :: proc (vertList:^[dynamic]shape_vertex2di64, indList:^[dynamic]u32, color:linalg.Vector4f32, pts:[][2]FixedDef, F:[4][3]FixedDef) {
        if len(pts) == 2 {
            return
        }
        start :u32 = u32(len(vertList))
        non_zero_append(vertList, shape_vertex2di64{
            uvw = {f32(to_f64(F[0][0])), f32(to_f64(F[0][1])), f32(to_f64(F[0][2]))},
            color = color,
        })
        non_zero_append(vertList, shape_vertex2di64{
            uvw = {f32(to_f64(F[1][0])), f32(to_f64(F[1][1])), f32(to_f64(F[1][2]))},
            color = color,
        })
        non_zero_append(vertList, shape_vertex2di64{
            uvw = {f32(to_f64(F[2][0])), f32(to_f64(F[2][1])), f32(to_f64(F[2][2]))},
            color = color,
        })
        non_zero_append(vertList, shape_vertex2di64{
            uvw = {f32(to_f64(F[3][0])), f32(to_f64(F[3][1])), f32(to_f64(F[3][2]))},
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
            if PointInTriangle(vertList[start + i].pos, vertList[indices[0]].pos, vertList[indices[1]].pos, vertList[indices[2]].pos) {
                for k:u32 = 0; k < 3; k += 1 {
                    non_zero_append(indList, indices[k])
                    non_zero_append(indList, indices[(k + 1)%3])
                    non_zero_append(indList, start + i)
                }
                return
            }
        }

        b := LinesIntersect(vertList[start].pos, vertList[start + 2].pos, vertList[start + 1].pos, vertList[start + 3].pos)
        if b {
            if length2_fixed(sub2_fixed(vertList[start + 2].pos, vertList[start].pos)).i < length2_fixed(sub2_fixed(vertList[start + 3].pos, vertList[start + 1].pos)).i {
                non_zero_append(indList, start, start + 1, start + 2, start, start + 2, start + 3)
            } else {
                non_zero_append(indList, start, start + 1, start + 3, start + 1, start + 2, start + 3)
            }
            return
        }
        b = LinesIntersect(vertList[start].pos, vertList[start + 3].pos, vertList[start + 1].pos, vertList[start + 2].pos)
        if b {
            if length2_fixed(sub2_fixed(vertList[start + 3].pos, vertList[start].pos)).i < length2_fixed(sub2_fixed(vertList[start + 2].pos, vertList[start + 1].pos)).i {
                non_zero_append(indList, start, start + 1, start + 3, start, start + 3, start + 2)
            } else {
                non_zero_append(indList, start, start + 1, start + 2, start + 2, start + 1, start + 3)
            }
            return
        }
        if length2_fixed(sub2_fixed(vertList[start + 1].pos, vertList[start].pos)).i < length2_fixed(sub2_fixed(vertList[start + 3].pos, vertList[start + 2].pos)).i {
            non_zero_append(indList, start, start + 2, start + 1, start, start + 1, start + 3)
        } else {
            non_zero_append(indList, start, start + 2, start + 3, start + 3, start + 2, start + 1)
        }
    }
    appendLine(vertList, indList, color, pts[:len(pts)], F)

    return nil
}

cvt_raw_shapei64_to_raw_shape :: proc(raw64:raw_shapei64, allocator := context.allocator) ->  (res:raw_shape, err:runtime.Allocator_Error) #optional_allocator_error {
	res.vertices = make_non_zeroed_slice([]shape_vertex2d, len(raw64.vertices), allocator) or_return
	defer if err != nil do delete(res.vertices, allocator)
	res.indices = make_non_zeroed_slice([]u32, len(raw64.indices), allocator) or_return
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
	res.vertices = make_non_zeroed_slice([]shape_vertex2di64, len(raw.vertices), allocator) or_return
	defer if err != nil do delete(res.vertices, allocator)
	res.indices = make_non_zeroed_slice([]u32, len(raw.indices), allocator) or_return
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
		nodes = make_non_zeroed_slice([]shape_nodei64, len(poly.nodes), allocator) or_return
	}

	for n, i in poly.nodes {
		poly64.nodes[i].pts = make_non_zeroed_slice([][2]FixedDef, len(n.pts), allocator)
		for p, j in n.pts {
            fixed.init_from_f64(&poly64.nodes[i].pts[j].x, f64(p.x))
            fixed.init_from_f64(&poly64.nodes[i].pts[j].y, f64(p.y))
		}

		if n.curve_pts_ids != nil {
			poly64.nodes[i].curve_pts_ids = make_non_zeroed_slice([]u32, len(n.curve_pts_ids), allocator)
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
		nodes = make_non_zeroed_slice([]shape_node, len(poly.nodes), allocator) or_return
	}

	for n, i in poly.nodes {
		poly32.nodes[i].pts = make_non_zeroed_slice([]linalg.Vector2f32, len(n.pts), allocator)
		for p, j in n.pts {
			poly32.nodes[i].pts[j].x = f32(fixed.to_f64(p.x))
			poly32.nodes[i].pts[j].y = f32(fixed.to_f64(p.y))
		}

		if n.curve_pts_ids != nil {
			poly32.nodes[i].curve_pts_ids = make_non_zeroed_slice([]u32, len(n.curve_pts_ids), allocator)
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
		nodes = make_non_zeroed_slice([]shape_nodei64, len(poly.nodes), context.temp_allocator) or_return
	}

	for n, i in poly.nodes {
		poly64.nodes[i].pts = make_non_zeroed_slice([][2]FixedDef, len(n.pts), context.temp_allocator)
		for p, j in n.pts {
			fixed.init_from_f64(&poly64.nodes[i].pts[j].x, f64(p.x))
			fixed.init_from_f64(&poly64.nodes[i].pts[j].y, f64(p.y))
		}

		if n.curve_pts_ids != nil {
			poly64.nodes[i].curve_pts_ids = make_non_zeroed_slice([]u32, len(n.curve_pts_ids), context.temp_allocator)
			mem.copy_non_overlapping(raw_data(poly64.nodes[i].curve_pts_ids), raw_data(n.curve_pts_ids), len(n.curve_pts_ids) * size_of(u32))
		} else {
			poly64.nodes[i].curve_pts_ids = nil
		}

		poly64.nodes[i].is_closed = n.is_closed
        fixed.init_from_f64(&poly64.nodes[i].thickness, f64(n.thickness))
		poly64.nodes[i].color = n.color
		poly64.nodes[i].stroke_color = n.stroke_color
	}

	res64 := shapes_compute_polygoni64(poly64, context.temp_allocator) or_return

	res.vertices = make_non_zeroed_slice([]shape_vertex2d, len(res64.vertices), allocator) or_return
	defer if err != nil do delete(res.vertices, allocator)
	res.indices = make_non_zeroed_slice([]u32, len(res64.indices), allocator) or_return
	defer if err != nil do delete(res.indices, allocator)// not working

	for v, i in res64.vertices {
        res.vertices[i].pos.x = f32(fixed.to_f64(v.pos.x))
		res.vertices[i].pos.y = f32(fixed.to_f64(v.pos.y))

		res.vertices[i].color = v.color
		res.vertices[i].uvw = v.uvw
	}

	mem.copy_non_overlapping(raw_data(res.indices), raw_data(res64.indices), len(res64.indices) * size_of(u32))
	return
}

@private CURVE_STRUCT :: struct {
	start:[2]FixedDef,
	ctl0:[2]FixedDef,
	ctl1:[2]FixedDef,
	end:[2]FixedDef,
	type:curve_type,
}

shapes_compute_polygoni64 :: proc(poly:shapesi64, allocator := context.allocator) -> (res:raw_shapei64, err:shape_error = nil) {
    vertList:[dynamic]shape_vertex2di64 = make([dynamic]shape_vertex2di64, context.temp_allocator)
    indList:[dynamic]u32 = make([dynamic]u32, context.temp_allocator)

    shapes_compute_polygon_in :: proc(vertList:^[dynamic]shape_vertex2di64, indList:^[dynamic]u32, poly:shapesi64, allocator : runtime.Allocator) -> (err:shape_error = nil) {	
        for node, nidx in poly.nodes {
			if node.color.a > 0 {
                non_curves:[dynamic][2]FixedDef = make([dynamic][2]FixedDef, context.temp_allocator)
			    curves:[dynamic]CURVE_STRUCT = make([dynamic]CURVE_STRUCT, context.temp_allocator)

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
                for cur,i in curves {

                } 
			}
		}
		return
	}
	return
}

poly_transform_matrix :: proc "contextless" (inout_poly: ^shapes, F: linalg.Matrix4x4f32) {
	for &node in inout_poly.nodes {
		for &pts in node.pts {
			out := linalg.mul(F, linalg.Vector4f32{pts.x, pts.y, 0, 1})
			pts = linalg.Vector2f32{out.x, out.y} / out.w
		}
	}
}