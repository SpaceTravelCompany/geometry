package geometry

import "base:intrinsics"
import "core:math"
import "core:mem"
import "core:math/fixed"
import "core:math/linalg"

Recti32 :: Rect_(i32)
Rectu32 :: Rect_(u32)
Rectf32 :: Rect_(f32)

center_pt_pos :: enum {
    Center,
    Left,
    Right,
    TopLeft,
    Top,
    TopRight,
    BottomLeft,
    Bottom,
    BottomRight,
}

Rect_ :: struct($T: typeid) where intrinsics.type_is_numeric(T) {
	left: T,
	right: T,
	top: T,
	bottom: T,
}

// Rect_Init2 :: #force_inline proc "contextless"(x: $T, y: T, width: T, height: T) -> Rect_(T) {
// }

Rect_Init :: #force_inline proc "contextless"(left: $T, right: T, top: T, bottom: T) -> Rect_(T) {
	res: Rect_(T)
	res.left = left
	res.right = right
	res.top = top
	res.bottom = bottom
	return res
}

//returns a Rectf32 whose centre linalg.Vector2f32 is the position
// Rect_GetFromCenter :: #force_inline proc "contextless" (_pos: [2]$T, _size: [2]T) -> Rect_(T)  {
// 	res: Rect_(T)
// 	res.pos = _pos - _size / 2
// 	res.size = _size
// 	return res
// }

Check_Rect :: #force_inline proc "contextless" (_pts:[4][$N]f32) -> bool where N >= 2 {
	return !(_pts[0].y != _pts[1].y || _pts[2].y != _pts[3].y || _pts[0].x != _pts[2].x || _pts[1].x != _pts[3].x)
}

/*
Multiplies a rectangle by a matrix. FAILED IF RESULT POINTS IS NOT RECTANGLE.
Inputs:
- _r: Rectangle to multiply
- _mat: matrix44 to multiply

Returns:
- Rectangle multiplied by the matrix
- bool: true if the result is a rectangle, false if the result is a polygon
*/
Rect_MulMatrix :: proc "contextless" (_r: Rectf32, _mat: linalg.Matrix4x4f32) -> (Rectf32, bool) {
	tps := __Rect_MulMatrix(_r, _mat)

	if Check_Rect(tps) do return {}, false

	return Rectf32{
		left = tps[0].x,
		right = tps[3].x,
		top = tps[0].y,
		bottom = tps[3].y,
	}, true
}
Rect_DivMatrix :: proc "contextless" (_r: Rectf32, _mat: linalg.Matrix4x4f32) -> (Rectf32, bool) {
	// Apply inverse transform (Rectf32 / M == Rectf32 * inverse(M))
	return Rect_MulMatrix(_r, linalg.inverse(_mat))
}

@private __Rect_MulMatrix :: proc "contextless" (_r: Rectf32, _mat: linalg.Matrix4x4f32) -> [4]linalg.Vector4f32 {
	// Transform 4 corners, then rebuild AABB.
	x0 := _r.left
	y0 := _r.top
	x1 := _r.right
	y1 := _r.bottom

	p0 := linalg.Vector4f32{x0, y0, 0, 1}
	p1 := linalg.Vector4f32{x1, y0, 0, 1}
	p2 := linalg.Vector4f32{x0, y1, 0, 1}
	p3 := linalg.Vector4f32{x1, y1, 0, 1}

	t0 := linalg.mul(_mat, p0)
	t1 := linalg.mul(_mat, p1)
	t2 := linalg.mul(_mat, p2)
	t3 := linalg.mul(_mat, p3)

	// Homogeneous divide if needed (safe for ortho/TRS where w == 1).
	tx0, ty0 := t0.x, t0.y
	tx1, ty1 := t1.x, t1.y
	tx2, ty2 := t2.x, t2.y
	tx3, ty3 := t3.x, t3.y

	if t0.w != 0 { tx0 /= t0.w; ty0 /= t0.w }
	if t1.w != 0 { tx1 /= t1.w; ty1 /= t1.w }
	if t2.w != 0 { tx2 /= t2.w; ty2 /= t2.w }
	if t3.w != 0 { tx3 /= t3.w; ty3 /= t3.w }

	return [4]linalg.Vector4f32{t0, t1, t2, t3}
}

/*
Multiplies a rectangle or a polygon by a matrix.
if input is a rectangle, and converted result is not a rectangle, it is converted to a polygon.
if input is a polygon, return the polygon.
if return value is the polygon, it is allocated using allocator.

Inputs:
- _a: AreaF(Rectangle or polygon) to multiply
- _mat: matrix44 to multiply
- allocator: Allocator to use

Returns:
- AreaF multiplied by the matrix
*/
Area_MulMatrix :: proc (_a: AreaF, _mat: linalg.Matrix4x4f32, allocator := context.allocator) -> AreaF {
	switch &n in _a {
	case Rectf32:
		res := __Rect_MulMatrix(n, _mat)
		if Check_Rect(res) {
			min_x := min(res[0].x, res[3].x)
			max_x := max(res[0].x, res[3].x)
			min_y := min(res[0].y, res[3].y)
			max_y := max(res[0].y, res[3].y)
			return Rectf32{
				left = min_x,
				right = max_x,
				top = max_y,
				bottom = min_y,
			}
		}
		res2 := make_non_zeroed_slice([][2]f32, 4, allocator)
		res2[0] = res[0].xy
		res2[1] = res[1].xy
		res2[2] = res[3].xy
		res2[3] = res[2].xy
		return res2
	case [][2]f32:
		return __Poly_MulMatrix(n, _mat, allocator)
	case ImageArea:
		panic_contextless("ImageArea: Available only for ImageButton\n")
	}
	return {}
}

@private __Poly_MulMatrix :: proc (_p:[][$N]f32, _mat: linalg.Matrix4x4f32, allocator := context.allocator) -> AreaF where N >= 2 {
	res: [][2]f32 = make_non_zeroed_slice([][2]f32, len(_p), allocator)
	for i in 0..<len(res) {
		when N == 4 {
			r := linalg.mul(_mat, _p[i])
		} else when N == 2 {
			r := linalg.mul(_mat, linalg.Vector4f32{_p[i].x, _p[i].y, 0.0, 1.0})
		} else when N == 3 {
			r := linalg.mul(_mat, linalg.Vector4f32{_p[i].x, _p[i].y, _p[i].z, 1.0})
		} else {
			#panic("not implemented")
		}
		if r.w != 0 { r.x /= r.w; r.y /= r.w }
		res[i] = r.xy
	}
	return res
}
Rect_LeftTop :: #force_inline proc "contextless" (_r: Rect_($T)) -> [2]T {
	return [2]T{_r.left, _r.top}
}
Rect_RightBottom :: #force_inline proc "contextless" (_r: Rect_($T)) -> [2]T {
	return [2]T{_r.right, _r.bottom}
}
Rect_And :: #force_inline proc "contextless" (_r1: Rect_($T), _r2: Rect_(T)) -> Rect_(T) #no_bounds_check {
	res: Rect_(T)
	res.left = max(_r1.left, _r2.left)
	res.right = min(_r1.right, _r2.right)
	if res.right < res.left do return {}

	r1_top := max(_r1.top, _r1.bottom)
	r1_bottom := min(_r1.top, _r1.bottom)
	r2_top := max(_r2.top, _r2.bottom)
	r2_bottom := min(_r2.top, _r2.bottom)

	y_top := min(r1_top, r2_top)
	y_bottom := max(r1_bottom, r2_bottom)

	if _r1.top >= _r1.bottom {
		res.top = y_top
		res.bottom = y_bottom
	} else {
		res.top = y_bottom
		res.bottom = y_top
	}
	return res
}
Rect_Or :: #force_inline proc "contextless" (_r1: Rect_($T), _r2: Rect_(T)) -> Rect_(T) #no_bounds_check {
	res: Rect_(T)

	res.left = min(_r1.left, _r2.left)
	res.right = max(_r1.right, _r2.right)

	r1_top := max(_r1.top, _r1.bottom)
	r1_bottom := min(_r1.top, _r1.bottom)
	r2_top := max(_r2.top, _r2.bottom)
	r2_bottom := min(_r2.top, _r2.bottom)

	y_top := max(r1_top, r2_top)
	y_bottom := min(r1_bottom, r2_bottom)

	if _r1.top >= _r1.bottom {
		res.top = y_top
		res.bottom = y_bottom
	} else {
		res.top = y_bottom
		res.bottom = y_top
	}
	return res
}
Rect_PointIn :: #force_inline proc "contextless" (_r: Rect_($T), p: [2]T) -> bool #no_bounds_check {
	return p.x >= _r.left && p.x <= _r.right && (_r.top > _r.bottom ? (p.y <= _r.top && p.y >= _r.bottom) : (p.y >= _r.top && p.y <= _r.bottom))
}

Rect_Move :: #force_inline proc "contextless" (_r: Rect_($T), p: [2]T) -> Rect_(T) #no_bounds_check {
	res: Rect_(T)
	res.left = _r.left + p.x
	res.top = _r.top + p.y
	res.right = _r.right + p.x
	res.bottom = _r.bottom + p.y
	return res
}

PointInTriangle :: proc "contextless" (p : [2]$T, a : [2]T, b : [2]T, c : [2]T) -> bool where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		x0 := c.x - a.x
		y0 := c.y - a.y
		x1 := b.x - a.x
		y1 := b.y - a.y
		x2 := p.x - a.x
		y2 := p.y - a.y

		dot00 := x0 * x0 + y0 * y0
		dot01 := x0 * x1 + y0 * y1
		dot02 := x0 * x2 + y0 * y2
		dot11 := x1 * x1 + y1 * y1
		dot12 := x1 * x2 + y1 * y2
		denominator := dot00 * dot11 - dot01 * dot01
		if (denominator == 0.0) do return false

		inverseDenominator := 1.0 / denominator
		u := (dot11 * dot02 - dot01 * dot12) * inverseDenominator
		v := (dot00 * dot12 - dot01 * dot02) * inverseDenominator

		return (u > 0.0) && (v > 0.0) && (u + v < 1.0)
	} else {
		F :: intrinsics.type_polymorphic_record_parameter_value(T, 1)

		x0 :T = fixed.sub(c.x, a.x)
		y0 :T = fixed.sub(c.y, a.y)
		x1 :T = fixed.sub(b.x, a.x)
		y1 :T = fixed.sub(b.y, a.y)
		x2 :T = fixed.sub(p.x, a.x)
		y2 :T = fixed.sub(p.y, a.y)

		dot00 :T = fixed.add(fixed.mul(x0, x0), fixed.mul(y0, y0))
		dot01 :T = fixed.add(fixed.mul(x0, x1), fixed.mul(y0, y1))
		dot02 :T = fixed.add(fixed.mul(x0, x2), fixed.mul(y0, y2))
		dot11 :T = fixed.add(fixed.mul(x1, x1), fixed.mul(y1, y1))
		dot12 :T = fixed.add(fixed.mul(x1, x2), fixed.mul(y1, y2))
		denominator :T = fixed.sub(fixed.mul(dot00, dot11), fixed.mul(dot01, dot01))
		if (denominator.i == 0) do return false

		inverseDenominator :T = fixed.div(T{i=1<<F}, denominator)
		u :T = fixed.mul(fixed.sub(fixed.mul(dot11, dot02), fixed.mul(dot01, dot12)), inverseDenominator)
		v :T = fixed.mul(fixed.sub(fixed.mul(dot00, dot12), fixed.mul(dot01, dot02)), inverseDenominator)

		return (u.i > 0) && (v.i > 0) && (u.i + v.i < 1<<F)
	}
}

PointInLine :: proc "contextless" (p:[2]$T, l0:[2]T, l1:[2]T) -> (bool, T) where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed){
	when intrinsics.type_is_float(T) {
		A := (l0.y - l1.y) / (l0.x - l1.x)
		B := l0.y - A * l0.x

		pY := A * p.x + B
		EP :: math.epsilon(T) * 18// about count float operations
		res := p.y >= pY - EP && p.y <= pY + EP
		t :T = 0.0
		if res {
			minX := min(l0.x, l1.x)
			maxX := max(l0.x, l1.x)
			t = (p.x - minX) / (maxX - minX)
		}

		return res &&
			p.x >= min(l0.x, l1.x) &&
			p.x <= max(l0.x, l1.x) &&
			p.y >= min(l0.y, l1.y) &&
			p.y <= max(l0.y, l1.y), t
	} else {
		A :T = fixed.div(fixed.sub(l0.y, l1.y), fixed.sub(l0.x, l1.x))
		B :T = l0.y - A * l0.x

		pY :T = A * p.x + B
		res :T = p.y >= pY - epsilon(T) && p.y <= pY + epsilon(T) 
		t :T = 0.0
		if res {
			minX :T = min(l0.x, l1.x)
			maxX :T = max(l0.x, l1.x)
			t = (p.x - minX) / (maxX - minX)
		}

		return res &&
			p.x.i >= min_fixed(l0.x, l1.x).i &&
			p.x.i <= max_fixed(l0.x, l1.x).i &&
			p.y.i >= min_fixed(l0.y, l1.y).i &&
			p.y.i <= max_fixed(l0.y, l1.y).i, t
	}
}

min_fixed :: proc "contextless" (v0:$T, v1:T) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	return v0.i < v1.i ? v0 : v1
}

max_fixed :: proc "contextless" (v0:$T, v1:T) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	return v0.i > v1.i ? v0 : v1
} 

PointDeltaInLine :: proc "contextless" (p:[2]$T, l0:[2]T, l1:[2]T) -> T where intrinsics.type_is_float(T) {
	A := (l0.y - l1.y) / (l0.x - l1.x)
	B := l0.y - A * l0.x

	pp := NearestPointBetweenPointAndLine(p, l0, l1)

	pY := A * pp.x + B
	t :T = 0.0
	minX := min(l0.x, l1.x)
	maxX := max(l0.x, l1.x)
	t = (p.x - minX) / (maxX - minX)

	return t
}

PointInVector :: proc "contextless" (p:[2]$T, v0:[2]T, v1:[2]T) -> (bool, T) where intrinsics.type_is_float(T) {
	a := v1.y - v0.y
	b := v0.x - v1.x
	c := v1.x * v0.y + v0.x * v1.y
	res := a * p.x + b * p.y + c
	return res == 0, res
}

PointLineLeftOrRight :: #force_inline proc "contextless" (p : [2]$T, l0 : [2]T, l1 : [2]T) -> T where intrinsics.type_is_float(T) {
	return (l1.x - l0.x) * (p.y - l0.y) - (p.x - l0.x) * (l1.y - l0.y)
}

PointInPolygon :: proc "contextless" (p: [2]$T, polygon:[][2]T) -> bool where intrinsics.type_is_float(T) {
	crossProduct :: proc "contextless" (p1: [2]$T, p2 : [2]T, p3 : [2]T) -> T {
		return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)
	}
	isPointOnSegment :: proc "contextless" (p: [2]$T, p1 : [2]T, p2 : [2]T) -> bool {
		return crossProduct(p1, p2, p) == 0 && p.x >= min(p1.x, p2.x) && p.x <= max(p1.x, p2.x) && p.y >= min(p1.y, p2.y) && p.y <= max(p1.y, p2.y)
	}
	windingNumber := 0
	for i in  0..<len(polygon) {
		p1 := polygon[i]
        p2 := polygon[(i + 1) % len(polygon)]

        if (isPointOnSegment(p, p1, p2)) {
            return false
        }

        if (p1.y <= p.y) {
            if (p2.y > p.y && crossProduct(p1, p2, p) > 0) {
                windingNumber += 1
            }
        } else {
            if (p2.y <= p.y && crossProduct(p1, p2, p) < 0) {
                windingNumber -= 1
            }
        }
	}
	return windingNumber != 0
}

CenterPointInPolygon :: proc "contextless" (polygon : [][2]$T) -> [2]T where intrinsics.type_is_float(T) {
	area :f32 = 0
	p : [2]T = {0,0}
	for i in 0..<len(polygon) {
		j := (i + 1) % len(polygon)
		factor := linalg.vector_cross2(polygon[i], polygon[j])
		area += factor
		p = (polygon[i] + polygon[j]) * splat_2(factor) + p
	}
	area = area / 2 * 6
	p *= splat_2(1 / area)
	return p
}

GetPolygonOrientation :: proc "contextless" (polygon : [][2]$T) -> PolyOrientation where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		res : T = 0
		for i in 0..<len(polygon) {
			j := (i + 1) % len(polygon)
			factor := (polygon[j].x - polygon[i].x) * (polygon[j].y + polygon[i].y)
			res += factor
		}
		return res > 0 ? .Clockwise : .CounterClockwise
	} else {
		res :T = {}
		for i in 0..<len(polygon) {
			j := (i + 1) % len(polygon)
			factor := (polygon[j].x.i - polygon[i].x.i) * (polygon[j].y.i + polygon[i].y.i)
			res.i += factor
		}
		return res.i > 0 ? .Clockwise : .CounterClockwise
	}
}

LineInPolygon :: proc "contextless" (a : [2]$T, b : [2]T, polygon : [][2]T, checkInsideLine := true) -> bool where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	//Points a, b must all be inside the polygon so that line a, b and polygon line segments do not intersect, so b does not need to be checked.
	if checkInsideLine && PointInPolygon(a, polygon) do return true

	res : [2]T
	ok:bool
	for i in 0..<len(polygon) {
		j := (i + 1) % len(polygon)
		ok, res = LinesIntersect(polygon[i], polygon[j], a, b)
		if ok {
			when intrinsics.type_is_float(T) {
				if a == res || b == res do continue
			} else {
				if a.i == res.i || b.i == res.i do continue
			}
			return true
		}
	}
	
	return false
}

LinesIntersect :: proc "contextless" (a1 : [2]$T, a2 : [2]T, b1: [2]T, b2 : [2]T) -> bool where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	Orientation :: proc "contextless" (p1 : [2]$T, p2 : [2]T, p3: [2]T) -> int {
		when intrinsics.type_is_float(T) {
			crossProduct := (p2.y - p1.y) * (p3.x - p2.x) - (p3.y - p2.y) * (p2.x - p1.x)
			return (crossProduct < 0.0) ? -1 : ((crossProduct > 0.0) ? 1 : 0)
		} else {
			crossProduct := fixed.sub(fixed.mul(fixed.sub(p2.y, p1.y) * fixed.sub(p3.x, p2.x)), fixed.mul(fixed.sub(p3.y, p2.y), fixed.sub(p2.x, p1.x)))
			return (crossProduct.i < 0) ? -1 : ((crossProduct.i > 0) ? 1 : 0)
		}
	}
	return (Orientation(a1, a2, b1) != Orientation(a1, a2, b2) && Orientation(b1, b2, a1) != Orientation(b1, b2, a2))
}

// based on http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/, edgeA => "line a", edgeB => "line b"
LinesIntersect2 :: proc "contextless" (a1 : [2]$T, a2 : [2]T, b1: [2]T, b2 : [2]T) -> (bool, bool, [2]T) where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		den :T = (b2.y - b1.y) * (a2.x - a1.x) - (b2.x - b1.x) * (a2.y - a1.y)
		if den == 0.0 {
			return false, false, {}
		}

		ua := ((b2.x - b1.x) * (a1.y - b1.y) - (b2.y - b1.y) * (a1.x - b1.x)) / den
		ub := ((a2.x - a1.x) * (a1.y - b1.y) - (a2.y - a1.y) * (a1.x - b1.x)) / den

		return true, ua >= 0.0 && ub >= 0.0 && ua <= 1.0 && ub <= 1.0, [2]T{a1.x + ua * (a2.x - a1.x), a1.y + ua * (a2.y - a1.y)}
	} else {
		den :T = fixed.mul(fixed.sub(b2.y, b1.y), fixed.sub(a2.x, a1.x)) - fixed.mul(fixed.sub(b2.x, b1.x), fixed.sub(a2.y, a1.y))
		if den.i == 0 {
			return false, false, {}
		}

		ua := fixed.div(fixed.sub(fixed.mul(fixed.sub(b2.x, b1.x), fixed.sub(a1.y, b1.y)), fixed.mul(fixed.sub(b2.y, b1.y), fixed.sub(a1.x, b1.x))), den)
		ub := fixed.div(fixed.sub(fixed.mul(fixed.sub(a2.x, a1.x), fixed.sub(a1.y, b1.y)), fixed.mul(fixed.sub(a2.y, a1.y), fixed.sub(a1.x, b1.x))), den)

		F :: intrinsics.type_polymorphic_record_parameter_value(T, 1)
		return true, ua.i >= 0 && ub.i >= 0 && ua.i <= 1 << F && ub.i <= 1 << F, [2]T{fixed.add(a1.x, fixed.mul(ua, fixed.sub(a2.x, a1.x))), fixed.add(a1.y, fixed.mul(ua, fixed.sub(a2.y, a1.y)))}
	}
}

NearestPointBetweenPointAndLine :: proc "contextless" (p:[2]$T, l0:[2]T, l1:[2]T) -> [2]T where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		AB := l1 - l0
		AC := p - l0

		return l0 + AB * (linalg.vector_dot(AB, AC) / linalg.vector_dot(AB, AB))
	} else {
		AB := fixed.sub(l1, l0)
		AC := fixed.sub(p, l0)

		return fixed.add(l0, fixed.mul(AB, fixed.div(linalg.vector_dot_fixed(AB, AC), linalg.vector_dot_fixed(AB, AB))))
	}
}

Circle :: struct(T:typeid) where intrinsics.type_is_float(T) {
	p : [2]T,
	radius : T,
}

Circlef32 :: Circle(f32)
Circlef64 :: Circle(f64)

PolyOrientation :: enum {
	Clockwise,
	CounterClockwise,
}

OppPolyOrientation :: #force_inline proc "contextless" (ccw:PolyOrientation) -> PolyOrientation {
	return ccw == .Clockwise ? .CounterClockwise : .Clockwise
}

xy_mirror_point :: #force_inline proc "contextless" (pivot : [2]$T, target : [2]T) -> [2]T where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		return [2]T{2.0,2.0} * pivot - target
	} else {
		F :: intrinsics.type_polymorphic_record_parameter_value(T, 1)

		return [2]T{fixed.sub(fixed.mul(T{i=2<<F}, pivot.x), target.x), fixed.sub(fixed.mul(T{i=2<<F}, pivot.y), target.y)}
	}
}

ImageArea :: struct {}

Area :: union($T: typeid) where intrinsics.type_is_numeric(T)  {
	Rect_(T),
	[][2]T,
	ImageArea,//Available only for ImageButton
}

AreaF :: Area(f32)
AreaF64 :: Area(f64)
AreaI :: Area(i32)

Area_PointIn :: #force_inline proc "contextless" (area:Area($T), pt:[2]T) -> bool where intrinsics.type_is_numeric(T) {
	switch a in area {
	case Rect_(T):return Rect_PointIn(a, pt)
	case [][2]T:return PointInPolygon(pt, a)
	case ImageArea:panic_contextless("ImageArea: Available only for ImageButton\n")
	}
	return false
}


@(require_results)
srtc_2d_matrix :: proc "contextless" (t: linalg.Vector3f32, s: linalg.Vector2f32, r: f32, cp:linalg.Vector2f32) -> linalg.Matrix4x4f32 {
	pivot := linalg.matrix4_translate(linalg.Vector3f32{cp.x,cp.y,0.0})
	translation := linalg.matrix4_translate(t)
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x,s.y,1.0})
	return linalg.mul(translation, linalg.mul(rotation, linalg.mul(scale, pivot)))
}

@(require_results)
srt_2d_matrix :: proc "contextless" (t: linalg.Vector3f32, s: linalg.Vector2f32, r: f32) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x,s.y,1.0})
	return linalg.mul(translation, linalg.mul(rotation, scale))
}

@(require_results)
st_2d_matrix :: proc "contextless" (t: linalg.Vector3f32, s: linalg.Vector2f32) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x,s.y,1.0})
	return linalg.mul(translation, scale)
}

@(require_results)
rt_2d_matrix :: proc "contextless" (t: linalg.Vector3f32, r: f32) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
    rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	return linalg.mul(translation, rotation)
}


@(require_results)
t_2d_matrix :: proc "contextless" (t: linalg.Vector3f32) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
	return translation
}

@(require_results)
src_2d_matrix :: proc "contextless" (s: linalg.Vector2f32, r: f32, cp:linalg.Vector2f32) -> linalg.Matrix4x4f32 {
	pivot := linalg.matrix4_translate(linalg.Vector3f32{cp.x,cp.y,0.0})
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x,s.y,1.0})
	return linalg.mul(rotation, linalg.mul(scale, pivot))
}

@(require_results)
sr_2d_matrix :: proc "contextless" (s: linalg.Vector2f32, r: f32) -> linalg.Matrix4x4f32 {
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x,s.y,1.0})
	return linalg.mul(rotation, scale)
}

@(require_results)
s_2d_matrix :: proc "contextless" (s: linalg.Vector2f32) -> linalg.Matrix4x4f32 {
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x,s.y,1.0})
	return scale
}

@(require_results)
r_2d_matrix :: proc "contextless" (r: f32) -> linalg.Matrix4x4f32 {
    rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	return rotation
}

@(require_results)
srt_2d_matrix2 :: proc "contextless" (t: linalg.Vector3f32, s: linalg.Vector2f32, r: f32, cp:linalg.Vector2f32) -> linalg.Matrix4x4f32 {
    if cp != {0.0, 0.0} {
        return srtc_2d_matrix(t,s,r,cp)
    }
    if r != 0.0 {
        if s != {1.0, 1.0} {
            return srt_2d_matrix(t,s,r)
        } else {
            return rt_2d_matrix(t,r)
        }
    }
    if s != {1.0, 1.0} {
        return st_2d_matrix(t,s)
    }
    return t_2d_matrix(t)
}

@(require_results)
sr_2d_matrix2 :: proc "contextless" (s: linalg.Vector2f32, r: f32, cp:linalg.Vector2f32) -> Maybe(linalg.Matrix4x4f32) {
    if cp != {0.0, 0.0} {
        return src_2d_matrix(s,r,cp)
    }
    if r != 0.0 {
        if s != {1.0, 1.0} {
            return sr_2d_matrix(s,r)
        } else {
            return r_2d_matrix(r)
        }
    }
    if s != {1.0, 1.0} {
        return s_2d_matrix(s)
    }
    return nil
}


@(require_results)
vector_dot_fixed :: proc "contextless" (a, b: $T/[$N]$E) -> (c: E) where intrinsics.type_is_specialization_of(E, fixed.Fixed) #no_bounds_check {
	#unroll for i in 0..<N {
		c = fixed.add(c, fixed.mul(a[i], b[i]))
	}
	return
}