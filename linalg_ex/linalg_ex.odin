package linalg_ex

import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:math/fixed"
import "core:math/linalg"
import "shared:utils_private/fixed_bcd"

import "shared:utils_private"


Recti32 :: Rect_(i32)
Rectu32 :: Rect_(u32)
Rectf32 :: Rect_(f32)

CenterPtPos :: enum {
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

PointInPolygonResult :: enum u8 {
	On,
	Outside,
	Inside,
}

Rect_ :: struct(
	$T: typeid
) where intrinsics.type_is_numeric(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed)
{
	left:   T,
	right:  T,
	top:    T,
	bottom: T,
}


Rect_Init :: #force_inline proc "contextless" (left: $T, right: T, top: T, bottom: T) -> Rect_(T) {
	res: Rect_(T)
	res.left = left
	res.right = right
	res.top = top
	res.bottom = bottom
	return res
}

Check_Rect :: #force_inline proc "contextless" (
	_pts: [4][$N]$T,
) -> bool where N >= 2 && intrinsics.type_is_numeric(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	return(
		!(!NumEq(_pts[0].y, _pts[1].y) ||
			!NumEq(_pts[2].y, _pts[3].y) ||
			!NumEq(_pts[0].x, _pts[2].x) ||
			!NumEq(_pts[1].x, _pts[3].x)) \
	)
}

Rect_MulMatrix :: proc "contextless" (_r: Rectf32, _mat: linalg.Matrix4x4f32) -> (Rectf32, bool) {
	tps := __Rect_MulMatrix(_r, _mat)

	if Check_Rect(tps) do return {}, false

	return Rectf32{left = tps[0].x, right = tps[3].x, top = tps[0].y, bottom = tps[3].y}, true
}
Rect_DivMatrix :: proc "contextless" (_r: Rectf32, _mat: linalg.Matrix4x4f32) -> (Rectf32, bool) {
	// Apply inverse transform (Rectf32 / M == Rectf32 * inverse(M))
	return Rect_MulMatrix(_r, linalg.inverse(_mat))
}

@(private)
__Rect_MulMatrix :: proc "contextless" (
	_r: Rectf32,
	_mat: linalg.Matrix4x4f32,
) -> [4]linalg.Vector4f32 {
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

	if t0.w != 0 {tx0 /= t0.w; ty0 /= t0.w}
	if t1.w != 0 {tx1 /= t1.w; ty1 /= t1.w}
	if t2.w != 0 {tx2 /= t2.w; ty2 /= t2.w}
	if t3.w != 0 {tx3 /= t3.w; ty3 /= t3.w}

	return [4]linalg.Vector4f32{t0, t1, t2, t3}
}

/*
Multiplies a rectangle or a polygon by a matrix.
if input is a rectangle, and converted result is not a rectangle, it is converted to a polygon.
if input is a polygon, return the polygon.
if return value is the polygon, it is allocated using allocator.

Inputs:
- _a: Areaf32(Rectangle or polygon) to multiply
- _mat: matrix44 to multiply
- allocator: Allocator to use

Returns:
- Areaf32 multiplied by the matrix
*/
Area_MulMatrix :: proc(
	_a: Areaf32,
	_mat: linalg.Matrix4x4f32,
	allocator := context.allocator,
) -> Areaf32 {
	switch &n in _a {
	case Rectf32:
		res := __Rect_MulMatrix(n, _mat)
		if Check_Rect(res) {
			min_x := min(res[0].x, res[3].x)
			max_x := max(res[0].x, res[3].x)
			min_y := min(res[0].y, res[3].y)
			max_y := max(res[0].y, res[3].y)
			return Rectf32{left = min_x, right = max_x, top = max_y, bottom = min_y}
		}
		res2 := utils_private.make_non_zeroed_slice([][2]f32, 4, allocator)
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

@(private)
__Poly_MulMatrix :: proc(
	_p: [][$N]f32,
	_mat: linalg.Matrix4x4f32,
	allocator := context.allocator,
) -> Areaf32 where N >=
	2 {
	res: [][2]f32 = utils_private.make_non_zeroed_slice([][2]f32, len(_p), allocator)
	for i in 0 ..< len(res) {
		when N == 4 {
			r := linalg.mul(_mat, _p[i])
		} else when N == 2 {
			r := linalg.mul(_mat, linalg.Vector4f32{_p[i].x, _p[i].y, 0.0, 1.0})
		} else when N == 3 {
			r := linalg.mul(_mat, linalg.Vector4f32{_p[i].x, _p[i].y, _p[i].z, 1.0})
		} else {
			#panic("not implemented")
		}
		if r.w != 0 {r.x /= r.w; r.y /= r.w}
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
Rect_And :: #force_inline proc "contextless" (
	_r1: Rect_($T),
	_r2: Rect_(T),
) -> Rect_(T) #no_bounds_check {
	res: Rect_(T)
	res.left = NumMax(_r1.left, _r2.left)
	res.right = NumMin(_r1.right, _r2.right)
	if NumLt(res.right, res.left) do return {}

	r1_top := NumMax(_r1.top, _r1.bottom)
	r1_bottom := NumMin(_r1.top, _r1.bottom)
	r2_top := NumMax(_r2.top, _r2.bottom)
	r2_bottom := NumMin(_r2.top, _r2.bottom)

	y_top := NumMin(r1_top, r2_top)
	y_bottom := NumMax(r1_bottom, r2_bottom)

	if NumGe(_r1.top, _r1.bottom) {
		res.top = y_top
		res.bottom = y_bottom
	} else {
		res.top = y_bottom
		res.bottom = y_top
	}
	return res
}
Rect_Or :: #force_inline proc "contextless" (
	_r1: Rect_($T),
	_r2: Rect_(T),
) -> Rect_(T) #no_bounds_check {
	res: Rect_(T)
	res.left = NumMin(_r1.left, _r2.left)
	res.right = NumMax(_r1.right, _r2.right)

	r1_top := NumMax(_r1.top, _r1.bottom)
	r1_bottom := NumMin(_r1.top, _r1.bottom)
	r2_top := NumMax(_r2.top, _r2.bottom)
	r2_bottom := NumMin(_r2.top, _r2.bottom)

	y_top := NumMax(r1_top, r2_top)
	y_bottom := NumMin(r1_bottom, r2_bottom)

	if NumGe(_r1.top, _r1.bottom) {
		res.top = y_top
		res.bottom = y_bottom
	} else {
		res.top = y_bottom
		res.bottom = y_top
	}

	return res
}
Rect_PointIn :: #force_inline proc "contextless" (
	_r: Rect_($T),
	p: [2]T,
) -> bool #no_bounds_check {
	when intrinsics.type_is_specialization_of(
		T,
		fixed_bcd.BCD,
	) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
		return(
			p.x.i >= _r.left.i &&
			p.x.i <= _r.right.i &&
			(_r.top.i > _r.bottom.i ? (p.y.i <= _r.top.i && p.y.i >= _r.bottom.i) : (p.y.i >= _r.top.i && p.y.i <= _r.bottom.i)) \
		)
	} else {
		return(
			p.x >= _r.left &&
			p.x <= _r.right &&
			(_r.top > _r.bottom ? (p.y <= _r.top && p.y >= _r.bottom) : (p.y >= _r.top && p.y <= _r.bottom)) \
		)
	}
}

// Polygon-polygon overlap: vertex containment or edge intersection.
PolygonOverlapsPolygon :: proc "contextless" (
	poly1, poly2: [][2]$T,
) -> bool where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	if len(poly1) < 3 || len(poly2) < 3 do return false
	for p in poly1 {
		if PointInPolygon(p, poly2) == .Inside do return true
	}
	for p in poly2 {
		if PointInPolygon(p, poly1) == .Inside do return true
	}
	for i in 0 ..< len(poly1) {
		a1 := poly1[i]
		b1 := poly1[(i + 1) % len(poly1)]
		for j in 0 ..< len(poly2) {
			a2 := poly2[j]
			b2 := poly2[(j + 1) % len(poly2)]
			if LinesIntersect3(a1, b1, a2, b2) == .intersect do return true
		}
	}
	return false
}

Rect_Move :: #force_inline proc "contextless" (
	_r: Rect_($T),
	p: [2]T,
) -> Rect_(T) #no_bounds_check {
	res: Rect_(T)

	when intrinsics.type_is_specialization_of(
		T,
		fixed_bcd.BCD,
	) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
		res.left = T {
			i = _r.left.i + p.x.i,
		}
		res.top = T {
			i = _r.top.i + p.y.i,
		}
		res.right = T {
			i = _r.right.i + p.x.i,
		}
		res.bottom = T {
			i = _r.bottom.i + p.y.i,
		}
	} else {
		res.left = _r.left + p.x
		res.top = _r.top + p.y
		res.right = _r.right + p.x
		res.bottom = _r.bottom + p.y
	}
	return res
}

PointInTriangle :: proc "contextless" (
	p: [2]$T,
	a: [2]T,
	b: [2]T,
	c: [2]T,
) -> bool where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	zero := NumConst(0, T)
	x0 := NumSub(c.x, a.x)
	y0 := NumSub(c.y, a.y)
	x1 := NumSub(b.x, a.x)
	y1 := NumSub(b.y, a.y)
	x2 := NumSub(p.x, a.x)
	y2 := NumSub(p.y, a.y)

	dot00 := NumAdd(NumMul(x0, x0), NumMul(y0, y0))
	dot01 := NumAdd(NumMul(x0, x1), NumMul(y0, y1))
	dot02 := NumAdd(NumMul(x0, x2), NumMul(y0, y2))
	dot11 := NumAdd(NumMul(x1, x1), NumMul(y1, y1))
	dot12 := NumAdd(NumMul(x1, x2), NumMul(y1, y2))
	denominator := NumSub(NumMul(dot00, dot11), NumMul(dot01, dot01))
	if NumEq(denominator, zero) do return false

	u := NumSub(NumMul(dot11, dot02), NumMul(dot01, dot12))
	v := NumSub(NumMul(dot00, dot12), NumMul(dot01, dot02))

	if NumGt(denominator, zero) {
		return NumGt(u, zero) && NumGt(v, zero) && NumLt(NumAdd(u, v), denominator)
	}
	return NumLt(u, zero) && NumLt(v, zero) && NumGt(NumAdd(u, v), denominator)
}

epsilon :: proc "contextless" ($T: typeid) -> T where intrinsics.type_is_float(T) {
	when T == f32 {
		return T(math.F32_EPSILON)
	} else when T == f64 {
		return T(math.F64_EPSILON)
	} else {
		return T(math.F16_EPSILON)
	}
}

PointInLine :: proc "contextless" (
	p: [2]$T,
	l0: [2]T,
	l1: [2]T,
) -> (
	bool,
	T,
) where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	min_x := NumMin(l0.x, l1.x)
	max_x := NumMax(l0.x, l1.x)
	min_y := NumMin(l0.y, l1.y)
	max_y := NumMax(l0.y, l1.y)
	a := NumDiv(NumSub(l0.y, l1.y), NumSub(l0.x, l1.x))
	b := NumSub(l0.y, NumMul(a, l0.x))
	p_y := NumAdd(NumMul(a, p.x), b)
	in_bbox := NumGe(p.x, min_x) && NumLe(p.x, max_x) && NumGe(p.y, min_y) && NumLe(p.y, max_y)
	t := NumDiv(NumSub(p.x, min_x), NumSub(max_x, min_x))

	when intrinsics.type_is_float(T) {
		ep: T = epsilon(T) * NumConst(10, T)
		on_line := NumLe(NumAbs(NumSub(p.y, p_y)), ep)
		return on_line && in_bbox, t
	} else {
		on_line := NumEq(p.y, p_y)
		return on_line && in_bbox, t
	}
}

min_fixed :: proc "contextless" (
	v0: $T,
	v1: T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	return v0.i < v1.i ? v0 : v1
}

max_fixed :: proc "contextless" (
	v0: $T,
	v1: T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	return v0.i > v1.i ? v0 : v1
}

SubdivLine :: proc "contextless" (
	pts: [2][2]$T,
	subdiv: T,
) -> (
	pt01: [2]T,
) where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	when intrinsics.type_is_float(T) {
		pt01 = linalg.lerp(pts[0], pts[1], subdiv)
	} else {
		subdiv2 := splat_2_fixed(subdiv)
		pt01 = lerp_fixed(pts[0], pts[1], subdiv2)
	}
	return
}

SubdivQuadraticBezier :: proc "contextless" (
	pts: [3][2]$T,
	subdiv: T,
) -> (
	pt01, pt012, pt12: [2]T,
) where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	when intrinsics.type_is_float(T) {
		pt01 = linalg.lerp(pts[0], pts[1], subdiv)
		pt12 = linalg.lerp(pts[1], pts[2], subdiv)
		pt012 = linalg.lerp(pt01, pt12, subdiv)
	} else {
		subdiv2 := splat_2_fixed(subdiv)
		pt01 = lerp_fixed(pts[0], pts[1], subdiv2)
		pt12 = lerp_fixed(pts[1], pts[2], subdiv2)
		pt012 = lerp_fixed(pt01, pt12, subdiv2)
	}
	return
}

SubdivCubicBezier :: proc "contextless" (
	pts: [4][2]$T,
	subdiv: T,
) -> (
	c0, c1, m, d0, d1: [2]T,
) where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	when intrinsics.type_is_float(T) {
		subdiv2 := subdiv
		lerp_ :: linalg.lerp
	} else {
		subdiv2 := splat_2_fixed(subdiv)
		lerp_ :: lerp_fixed
	}
	p01 := lerp_(pts[0], pts[1], subdiv2)
	p12 := lerp_(pts[1], pts[2], subdiv2)
	p23 := lerp_(pts[2], pts[3], subdiv2)

	p012 := lerp_(p01, p12, subdiv2)
	p123 := lerp_(p12, p23, subdiv2)

	c0 = p01
	c1 = p012
	m = lerp_(p012, p123, subdiv2)
	d0 = p123
	d1 = p23
	return
}

// Fixed-point version of linalg.lerp: result = a + t*(b - a). t is typically in [0, 1].
lerp_fixed :: proc "contextless" (
	a, b, t: $T,
) -> T where intrinsics.type_is_specialization_of(intrinsics.type_elem_type(T), fixed.Fixed) ||
	intrinsics.type_is_specialization_of(
		intrinsics.type_elem_type(T),
		fixed_bcd.BCD,
	) #no_bounds_check {
	when intrinsics.type_is_array(T) {
		res: T
		#unroll for i in 0 ..< len(T) {
			res[i] = lerp_fixed(a[i], b[i], t[i])
		}
		return res
	} else {
		when intrinsics.type_is_specialization_of(intrinsics.type_elem_type(T), fixed.Fixed) {

			return fixed.add(a, fixed.mul(t, fixed.sub(b, a)))
		} else {
			return fixed_bcd.add(a, fixed_bcd.mul(t, fixed_bcd.sub(b, a)))
		}
	}
}

PointDeltaInLine :: proc "contextless" (
	p: [2]$T,
	l0: [2]T,
	l1: [2]T,
) -> T where intrinsics.type_is_float(T) {
	pp := NearestPointBetweenPointAndLine(p, l0, l1)
	t := linalg.lerp(l0.x, l1.x, pp.x)
	return t
}

PointInVector :: proc "contextless" (
	p: [2]$T,
	v0: [2]T,
	v1: [2]T,
) -> (
	bool,
	T,
) where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	zero := NumConst(0, T)
	a := NumSub(v1.y, v0.y)
	b := NumSub(v0.x, v1.x)
	c := NumAdd(NumMul(v1.x, v0.y), NumMul(v0.x, v1.y))
	res := NumAdd(NumAdd(NumMul(a, p.x), NumMul(b, p.y)), c)
	return NumEq(res, zero), res
}

PointLineLeftOrRight :: #force_inline proc "contextless" (
	p: [2]$T,
	l0: [2]T,
	l1: [2]T,
) -> T where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	return NumSub(
		NumMul(NumSub(l1.x, l0.x), NumSub(p.y, l0.y)),
		NumMul(NumSub(p.x, l0.x), NumSub(l1.y, l0.y)),
	)
}

PointInPolygon :: proc "contextless" (
	p: [2]$T,
	polygon: [][2]T,
) -> PointInPolygonResult where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	isPointOnSegment :: proc "contextless" (p: [2]$T, p1: [2]T, p2: [2]T) -> bool {
		return(
			CrossProductSign(p1, p2, p) == 0 &&
			NumGe(p.x, NumMin(p1.x, p2.x)) &&
			NumLe(p.x, NumMax(p1.x, p2.x)) &&
			NumGe(p.y, NumMin(p1.y, p2.y)) &&
			NumLe(p.y, NumMax(p1.y, p2.y)) \
		)
	}
	windingNumber := 0
	for i in 0 ..< len(polygon) {
		p1 := polygon[i]
		p2 := polygon[(i + 1) % len(polygon)]
		if isPointOnSegment(p, p1, p2) do return .On

		if NumLe(p1.y, p.y) {
			if NumGt(p2.y, p.y) && CrossProductSign(p1, p2, p) > 0 do windingNumber += 1
		} else {
			if NumLe(p2.y, p.y) && CrossProductSign(p1, p2, p) < 0 do windingNumber -= 1
		}
	}
	return windingNumber != 0 ? .Inside : .Outside
}

vector_cross2_fixed :: #force_inline proc "contextless" (
	a, b: [2]$E,
) -> E where intrinsics.type_is_specialization_of(E, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(E, fixed_bcd.BCD) {
	return NumSub(NumMul(a.x, b.y), NumMul(a.y, b.x))
}

vector_cross2_bcd :: #force_inline proc "contextless" (
	a, b: [2]$E,
) -> E where intrinsics.type_is_specialization_of(E, fixed_bcd.BCD) {
	return fixed_bcd.sub(fixed_bcd.mul(a.x, b.y), fixed_bcd.mul(a.y, b.x))
}

CenterPointInPolygon :: proc "contextless" (
	polygon: [][2]$T,
) -> [2]T where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	when intrinsics.type_is_float(T) {
		area: f32 = 0
		p: [2]T = {0, 0}
		for i in 0 ..< len(polygon) {
			j := (i + 1) % len(polygon)
			factor := linalg.vector_cross2(polygon[i], polygon[j])
			area += factor
			p = (polygon[i] + polygon[j]) * math.splat_2(factor) + p
		}
		area = area / 2 * 6
		p /= math.splat_2(area)
		return p
	} else {
		three_t := NumConst(3, T)
		area: T = {}
		p: [2]T = {}
		for i in 0 ..< len(polygon) {
			j := (i + 1) % len(polygon)
			factor := vector_cross2_fixed(polygon[i], polygon[j])
			area = NumAdd(area, factor)
			sum_pt := [2]T{NumAdd(polygon[i].x, polygon[j].x), NumAdd(polygon[i].y, polygon[j].y)}
			p.x = NumAdd(p.x, NumMul(sum_pt.x, factor))
			p.y = NumAdd(p.y, NumMul(sum_pt.y, factor))
		}
		area = NumMul(area, three_t)
		p.x = NumDiv(p.x, area)
		p.y = NumDiv(p.y, area)
		return p
	}
}

GetPolygonOrientation :: proc "contextless" (
	polygon: [][2]$T,
) -> PolyOrientation where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	zero := NumConst(0, T)
	res: T = zero
	for i in 0 ..< len(polygon) {
		j := (i + 1) % len(polygon)
		factor := NumMul(NumSub(polygon[j].x, polygon[i].x), NumAdd(polygon[j].y, polygon[i].y))
		res = NumAdd(res, factor)
	}
	return NumGt(res, zero) ? .Clockwise : .CounterClockwise
}

LineInPolygon :: proc "contextless" (
	a: [2]$T,
	b: [2]T,
	polygon: [][2]T,
	checkInsideLine := true,
) -> bool where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	//Points a, b must all be inside the polygon so that line a, b and polygon line segments do not intersect, so b does not need to be checked.
	if checkInsideLine && PointInPolygon(a, polygon) do return true

	res: [2]T
	ok: bool
	for i in 0 ..< len(polygon) {
		j := (i + 1) % len(polygon)
		ok, res = LinesIntersect2(polygon[i], polygon[j], a, b)
		if ok == .intersect {
			same_a := NumEq(a.x, res.x) && NumEq(a.y, res.y)
			same_b := NumEq(b.x, res.x) && NumEq(b.y, res.y)
			if same_a || same_b do continue
			return true
		}
	}

	return false
}

IntersectKind :: enum u8 {
	none,
	collinear,
	intersect,
}

// based on http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/, edgeA => "line a", edgeB => "line b"
LinesIntersect2 :: proc "contextless" (
	a1: [2]$T,
	a2: [2]T,
	b1: [2]T,
	b2: [2]T,
	check_is_touching: bool = false,
) -> (
	IntersectKind,
	[2]T,
) where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	if check_is_touching {
		same_a_b1 := NumEq(a1.x, b1.x) && NumEq(a1.y, b1.y)
		same_a_b2 := NumEq(a1.x, b2.x) && NumEq(a1.y, b2.y)
		same_a2_b1 := NumEq(a2.x, b1.x) && NumEq(a2.y, b1.y)
		same_a2_b2 := NumEq(a2.x, b2.x) && NumEq(a2.y, b2.y)
		if same_a_b1 || same_a_b2 do return .none, a1
		else if same_a2_b1 || same_a2_b2 do return .none, a2
	}

	zero := NumConst(0, T)
	den: T = NumSub(
		NumMul(NumSub(b2.y, b1.y), NumSub(a2.x, a1.x)),
		NumMul(NumSub(b2.x, b1.x), NumSub(a2.y, a1.y)),
	)
	if NumEq(den, zero) {
		return .collinear, {}
	}

	ua := NumSub(
		NumMul(NumSub(b2.x, b1.x), NumSub(a1.y, b1.y)),
		NumMul(NumSub(b2.y, b1.y), NumSub(a1.x, b1.x)),
	)
	ub := NumSub(
		NumMul(NumSub(a2.x, a1.x), NumSub(a1.y, b1.y)),
		NumMul(NumSub(a2.y, a1.y), NumSub(a1.x, b1.x)),
	)

	res_den :=
		NumGt(den, zero) ? (NumGe(ua, zero) && NumGe(ub, zero) && NumLe(ua, den) && NumLe(ub, den)) : (NumGe(ua, den) && NumGe(ub, den) && NumLe(ua, zero) && NumLe(ub, zero))

	t := NumDiv(ua, den)
	return res_den ? .intersect : .none, [2]T{NumAdd(a1.x, NumMul(t, NumSub(a2.x, a1.x))), NumAdd(a1.y, NumMul(t, NumSub(a2.y, a1.y)))}
}

LinesIntersect3 :: proc "contextless" (
	a1: [2]$T,
	a2: [2]T,
	b1: [2]T,
	b2: [2]T,
	check_is_touching: bool = false,
) -> IntersectKind where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	same_a1_b1 := NumEq(a1.x, b1.x) && NumEq(a1.y, b1.y)
	same_a2_b1 := NumEq(a2.x, b1.x) && NumEq(a2.y, b1.y)
	same_a1_b2 := NumEq(a1.x, b2.x) && NumEq(a1.y, b2.y)
	same_a2_b2 := NumEq(a2.x, b2.x) && NumEq(a2.y, b2.y)
	if check_is_touching && (same_a1_b1 || same_a2_b1 || same_a1_b2 || same_a2_b2) do return .none

	zero := NumConst(0, T)
	den: T = NumSub(
		NumMul(NumSub(b2.y, b1.y), NumSub(a2.x, a1.x)),
		NumMul(NumSub(b2.x, b1.x), NumSub(a2.y, a1.y)),
	)
	if NumEq(den, zero) do return .collinear
	ua := NumSub(
		NumMul(NumSub(b2.x, b1.x), NumSub(a1.y, b1.y)),
		NumMul(NumSub(b2.y, b1.y), NumSub(a1.x, b1.x)),
	)
	ub := NumSub(
		NumMul(NumSub(a2.x, a1.x), NumSub(a1.y, b1.y)),
		NumMul(NumSub(a2.y, a1.y), NumSub(a1.x, b1.x)),
	)
	res_den :=
		NumGt(den, zero) ? (NumGe(ua, zero) && NumGe(ub, zero) && NumLe(ua, den) && NumLe(ub, den)) : (NumGe(ua, den) && NumGe(ub, den) && NumLe(ua, zero) && NumLe(ub, zero))
	return res_den ? .intersect : .none
}

NearestPointBetweenPointAndLine :: proc "contextless" (
	p: [2]$T,
	l0: [2]T,
	l1: [2]T,
) -> [2]T where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	when intrinsics.type_is_float(T) {
		AB := l1 - l0
		AC := p - l0

		return l0 + AB * (linalg.vector_dot(AB, AC) / linalg.vector_dot(AB, AB))
	} else {
		ab := [2]T{NumSub(l1.x, l0.x), NumSub(l1.y, l0.y)}
		ac := [2]T{NumSub(p.x, l0.x), NumSub(p.y, l0.y)}
		t := NumDiv(Vec2Dot(ab, ac), Vec2Dot(ab, ab))
		return [2]T{NumAdd(l0.x, NumMul(ab.x, t)), NumAdd(l0.y, NumMul(ab.y, t))}
	}
}

Circle :: struct(T: typeid) where intrinsics.type_is_float(T) {
	p:      [2]T,
	radius: T,
}

Circlef32 :: Circle(f32)
Circlef64 :: Circle(f64)

PolyOrientation :: enum {
	Clockwise,
	CounterClockwise,
}

OppPolyOrientation :: #force_inline proc "contextless" (ccw: PolyOrientation) -> PolyOrientation {
	return ccw == .Clockwise ? .CounterClockwise : .Clockwise
}

xy_mirror_point :: #force_inline proc "contextless" (
	pivot: [2]$T,
	target: [2]T,
) -> [2]T where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.Fixed) {
	when intrinsics.type_is_float(T) {
		return [2]T{2.0, 2.0} * pivot - target
	} else {
		when intrinsics.type_is_specialization_of(T, fixed.Fixed) {

			add :: fixed.add
			sub :: fixed.sub
			mul :: fixed.mul
			two_T :: T {
				i = 2 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
			}
		} else {
			add :: fixed_bcd.add
			sub :: fixed_bcd.sub
			mul :: fixed_bcd.mul
			two_T :: T {
				i = 2 * fixed_bcd._SCALE_TABLE[intrinsics.type_polymorphic_record_parameter_value(T, 0) - 1],
			}
		}

		return [2]T{sub(mul(two_T, pivot.x), target.x), sub(mul(two_T, pivot.y), target.y)}
	}
}

ImageArea :: struct {}

Area :: union($T: typeid) where intrinsics.type_is_numeric(T) {
	Rect_(T),
	[][2]T,
	ImageArea, //Available only for ImageButton
}

Areaf32 :: Area(f32)
Areaf64 :: Area(f64)
Areai32 :: Area(i32)

Area_PointIn :: #force_inline proc "contextless" (
	area: Area($T),
	pt: [2]T,
) -> bool where intrinsics.type_is_numeric(T) {
	switch a in area {
	case Rect_(T):
		return Rect_PointIn(a, pt)
	case [][2]T:
		return PointInPolygon(pt, a)
	case ImageArea:
		panic_contextless("ImageArea: Available only for ImageButton\n")
	}
	return false
}


// precondition: l1 <> l2
// Returns the shortest squared distance from pt to the segment [l1, l2].
@(require_results)
ShortestLength2Line :: proc "contextless" (
	pt, l1, l2: [2]$T,
) -> (
	T,
	T,
) where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	zero := NumConst(0, T)
	one := NumConst(1, T)
	dx := NumSub(l2.x, l1.x)
	dy := NumSub(l2.y, l1.y)
	ax := NumSub(pt.x, l1.x)
	ay := NumSub(pt.y, l1.y)
	q_num := NumAdd(NumMul(ax, dx), NumMul(ay, dy))
	denom := NumAdd(NumMul(dx, dx), NumMul(dy, dy))
	if NumLt(q_num, zero) {
		return NumAdd(NumMul(ax, ax), NumMul(ay, ay)), one
	} else if NumGt(q_num, denom) {
		bx := NumSub(pt.x, l2.x)
		by := NumSub(pt.y, l2.y)
		return NumAdd(NumMul(bx, bx), NumMul(by, by)), one
	} else {
		cross := NumSub(NumMul(ax, dy), NumMul(dx, ay))
		return NumMul(cross, cross), denom
	}
}


@(require_results)
srtc_2d_matrix :: proc "contextless" (
	t: linalg.Vector3f32,
	s: linalg.Vector2f32,
	r: f32,
	cp: linalg.Vector2f32,
) -> linalg.Matrix4x4f32 {
	pivot := linalg.matrix4_translate(linalg.Vector3f32{cp.x, cp.y, 0.0})
	translation := linalg.matrix4_translate(t)
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x, s.y, 1.0})
	return linalg.mul(translation, linalg.mul(rotation, linalg.mul(scale, pivot)))
}

@(require_results)
srt_2d_matrix :: proc "contextless" (
	t: linalg.Vector3f32,
	s: linalg.Vector2f32,
	r: f32,
) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x, s.y, 1.0})
	return linalg.mul(translation, linalg.mul(rotation, scale))
}

@(require_results)
st_2d_matrix :: proc "contextless" (
	t: linalg.Vector3f32,
	s: linalg.Vector2f32,
) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x, s.y, 1.0})
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
src_2d_matrix :: proc "contextless" (
	s: linalg.Vector2f32,
	r: f32,
	cp: linalg.Vector2f32,
) -> linalg.Matrix4x4f32 {
	pivot := linalg.matrix4_translate(linalg.Vector3f32{cp.x, cp.y, 0.0})
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x, s.y, 1.0})
	return linalg.mul(rotation, linalg.mul(scale, pivot))
}

@(require_results)
sr_2d_matrix :: proc "contextless" (s: linalg.Vector2f32, r: f32) -> linalg.Matrix4x4f32 {
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x, s.y, 1.0})
	return linalg.mul(rotation, scale)
}

@(require_results)
s_2d_matrix :: proc "contextless" (s: linalg.Vector2f32) -> linalg.Matrix4x4f32 {
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x, s.y, 1.0})
	return scale
}

@(require_results)
r_2d_matrix :: proc "contextless" (r: f32) -> linalg.Matrix4x4f32 {
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	return rotation
}

@(require_results)
srt_2d_matrix2 :: proc "contextless" (
	t: linalg.Vector3f32,
	s: linalg.Vector2f32,
	r: f32,
	cp: linalg.Vector2f32,
) -> linalg.Matrix4x4f32 {
	if cp != {0.0, 0.0} {
		return srtc_2d_matrix(t, s, r, cp)
	}
	if r != 0.0 {
		if s != {1.0, 1.0} {
			return srt_2d_matrix(t, s, r)
		} else {
			return rt_2d_matrix(t, r)
		}
	}
	if s != {1.0, 1.0} {
		return st_2d_matrix(t, s)
	}
	return t_2d_matrix(t)
}

@(require_results)
sr_2d_matrix2 :: proc "contextless" (
	s: linalg.Vector2f32,
	r: f32,
	cp: linalg.Vector2f32,
) -> Maybe(linalg.Matrix4x4f32) {
	if cp != {0.0, 0.0} {
		return src_2d_matrix(s, r, cp)
	}
	if r != 0.0 {
		if s != {1.0, 1.0} {
			return sr_2d_matrix(s, r)
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
splat_2_fixed :: #force_inline proc "contextless" (
	v: $T,
) -> [2]T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	return [2]T{v, v}
}

CvtQuadraticToCubic0 :: #force_inline proc "contextless" (
	_start: [2]$T,
	_control: [2]T,
) -> [2]T where intrinsics.type_is_float(T) {
	return [2]T {
		_start[0] + T(2.0 / 3.0) * (_control[0] - _start[0]),
		_start[1] + T(2.0 / 3.0) * (_control[1] - _start[1]),
	}
}
CvtQuadraticToCubic1 :: #force_inline proc "contextless" (
	_end: [2]$T,
	_control: [2]T,
) -> [2]T where intrinsics.type_is_float(T) {
	return CvtQuadraticToCubic0(_end, _control)
}

//(p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x) == (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x)
@(require_results)
CrossProductSign :: proc "contextless" (
	p1, p2, p3: [2]$T,
) -> int where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	a := NumSub(p2.x, p1.x)
	b := NumSub(p3.y, p2.y)
	c := NumSub(p2.y, p1.y)
	d := NumSub(p3.x, p2.x)
	ab := NumMul(a, b)
	cd := NumMul(c, d)
	if NumGt(ab, cd) do return 1
	if NumLt(ab, cd) do return -1
	return 0
}

DotProduct :: proc "contextless" (
	p1, p2, p3: [2]$T,
) -> T where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	a := NumSub(p2.x, p1.x)
	b := NumSub(p3.x, p2.x)
	c := NumSub(p2.y, p1.y)
	d := NumSub(p3.y, p2.y)
	return NumAdd(NumMul(a, b), NumMul(c, d))
}

// InCircleTest: returns determinant. Positive => D inside circle through A,B,C (when ABC is CCW).
InCircleTest :: proc "contextless" (
	ptA, ptB, ptC, ptD: [2]$T,
) -> T where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	m00 := NumSub(ptA.x, ptD.x)
	m01 := NumSub(ptA.y, ptD.y)
	m02 := NumAdd(NumMul(m00, m00), NumMul(m01, m01))
	m10 := NumSub(ptB.x, ptD.x)
	m11 := NumSub(ptB.y, ptD.y)
	m12 := NumAdd(NumMul(m10, m10), NumMul(m11, m11))
	m20 := NumSub(ptC.x, ptD.x)
	m21 := NumSub(ptC.y, ptD.y)
	m22 := NumAdd(NumMul(m20, m20), NumMul(m21, m21))
	return NumAdd(
		NumSub(
			NumMul(m00, NumSub(NumMul(m11, m22), NumMul(m21, m12))),
			NumMul(m10, NumSub(NumMul(m01, m22), NumMul(m21, m02))),
		),
		NumMul(m20, NumSub(NumMul(m01, m12), NumMul(m11, m02))),
	)
}

GetAngle :: proc(a, b, c: [2]$T) -> T where intrinsics.type_is_float(T) {
	//https://stackoverflow.com/a/3487062/359538
	abx := b.x - a.x
	aby := b.y - a.y
	bcx := b.x - c.x
	bcy := b.y - c.y
	dp := abx * bcx + aby * bcy
	cp := abx * bcy - aby * bcx
	return math.atan2(cp, dp) //range between -Pi and Pi
}

