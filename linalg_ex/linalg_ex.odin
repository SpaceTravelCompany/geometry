package linalg_ex

import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:math/fixed"
import "core:math/linalg"
import "shared:utils_private/fixed_bcd"
import "shared:utils_private/fixed_ex"

import "shared:utils_private"


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

PointInPolygonResult :: enum u8 {
	On,
	Outside,
	Inside,
}

Rect_ :: struct($T: typeid) where intrinsics.type_is_numeric(T) {
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
) -> bool where N >= 2 &&
	intrinsics.type_is_numeric(T) {
	return(
		!(_pts[0].y != _pts[1].y ||
			_pts[2].y != _pts[3].y ||
			_pts[0].x != _pts[2].x ||
			_pts[1].x != _pts[3].x) \
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

	when intrinsics.type_is_specialization_of(
		T,
		fixed_bcd.BCD,
	) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
		max_ :: max_fixed
		min_ :: min_fixed
	} else {
		max_ :: max
		min_ :: min
	}

	res.left = max_(_r1.left, _r2.left)
	res.right = min_(_r1.right, _r2.right)
	if res.right < res.left do return {}

	r1_top := max_(_r1.top, _r1.bottom)
	r1_bottom := min_(_r1.top, _r1.bottom)
	r2_top := max_(_r2.top, _r2.bottom)
	r2_bottom := min_(_r2.top, _r2.bottom)

	y_top := min_(r1_top, r2_top)
	y_bottom := max_(r1_bottom, r2_bottom)

	when intrinsics.type_is_specialization_of(
		T,
		fixed_bcd.BCD,
	) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
		if _r1.top.i >= _r1.bottom.i {
			res.top = y_top
			res.bottom = y_bottom
		} else {
			res.top = y_bottom
			res.bottom = y_top
		}
	} else {
		if _r1.top >= _r1.bottom {
			res.top = y_top
			res.bottom = y_bottom
		} else {
			res.top = y_bottom
			res.bottom = y_top
		}
	}
	return res
}
Rect_Or :: #force_inline proc "contextless" (
	_r1: Rect_($T),
	_r2: Rect_(T),
) -> Rect_(T) #no_bounds_check {
	res: Rect_(T)

	when intrinsics.type_is_specialization_of(
		T,
		fixed_bcd.BCD,
	) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
		max_ :: max_fixed
		min_ :: min_fixed
	} else {
		max_ :: max
		min_ :: min
	}

	res.left = min_(_r1.left, _r2.left)
	res.right = max_(_r1.right, _r2.right)

	r1_top := max_(_r1.top, _r1.bottom)
	r1_bottom := min_(_r1.top, _r1.bottom)
	r2_top := max_(_r2.top, _r2.bottom)
	r2_bottom := min_(_r2.top, _r2.bottom)

	y_top := max_(r1_top, r2_top)
	y_bottom := min_(r1_bottom, r2_bottom)

	when intrinsics.type_is_specialization_of(
		T,
		fixed_bcd.BCD,
	) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
		if _r1.top.i >= _r1.bottom.i {
			res.top = y_top
			res.bottom = y_bottom
		} else {
			res.top = y_bottom
			res.bottom = y_top
		}
	} else {
		if _r1.top >= _r1.bottom {
			res.top = y_top
			res.bottom = y_bottom
		} else {
			res.top = y_bottom
			res.bottom = y_top
		}
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

		u := (dot11 * dot02 - dot01 * dot12)
		v := (dot00 * dot12 - dot01 * dot02)

		if denominator > 0.0 do return (u > 0.0) && (v > 0.0) && (u + v < denominator)
		return (u < 0.0) && (v < 0.0) && (u + v > denominator)
	} else {
		when intrinsics.type_is_specialization_of(T, fixed.Fixed) {

			add :: fixed.add
			sub :: fixed.sub
			mul :: fixed.mul
		} else {
			add :: fixed_bcd.add
			sub :: fixed_bcd.sub
			mul :: fixed_bcd.mul
		}

		x0: T = sub(c.x, a.x)
		y0: T = sub(c.y, a.y)
		x1: T = sub(b.x, a.x)
		y1: T = sub(b.y, a.y)
		x2: T = sub(p.x, a.x)
		y2: T = sub(p.y, a.y)

		dot00: T = add(mul(x0, x0), mul(y0, y0))
		dot01: T = add(mul(x0, x1), mul(y0, y1))
		dot02: T = add(mul(x0, x2), mul(y0, y2))
		dot11: T = add(mul(x1, x1), mul(y1, y1))
		dot12: T = add(mul(x1, x2), mul(y1, y2))
		denominator: T = sub(mul(dot00, dot11), mul(dot01, dot01))
		if (denominator.i == 0) do return false

		u: T = sub(mul(dot11, dot02), mul(dot01, dot12))
		v: T = sub(mul(dot00, dot12), mul(dot01, dot02))

		if denominator.i > 0 do return (u.i > 0) && (v.i > 0) && (u.i + v.i < denominator.i)
		return (u.i < 0) && (v.i < 0) && (u.i + v.i > denominator.i)
	}
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
	when intrinsics.type_is_float(T) {
		A := (l0.y - l1.y) / (l0.x - l1.x)
		B := l0.y - A * l0.x
		pY := A * p.x + B

		EP: T = epsilon(T) * T(10.0)
		res := p.y >= pY - EP && p.y <= pY + EP
		minX := min(l0.x, l1.x)
		maxX := max(l0.x, l1.x)

		return res &&
			p.x >= min(l0.x, l1.x) &&
			p.x <= max(l0.x, l1.x) &&
			p.y >= min(l0.y, l1.y) &&
			p.y <= max(l0.y, l1.y),
			(p.x - minX) / (maxX - minX)
	} else {
		when intrinsics.type_is_specialization_of(T, fixed.Fixed) {

			add :: fixed.add
			sub :: fixed.sub
			mul :: fixed.mul
			div :: fixed.div
			one_T :: T {
				i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
			}
		} else {
			add :: fixed_bcd.add
			sub :: fixed_bcd.sub
			mul :: fixed_bcd.mul
			div :: fixed_bcd.div
			one_T :: T {
				i = 1 * fixed_bcd._SCALE_TABLE[intrinsics.type_polymorphic_record_parameter_value(T, 0) - 1],
			}
		}

		A: T = div(sub(l0.y, l1.y), sub(l0.x, l1.x))
		B: T = sub(l0.y, mul(A, l0.x))
		pY: T = add(mul(A, p.x), B)
		minX: T = min_fixed(l0.x, l1.x)
		maxX: T = max_fixed(l0.x, l1.x)

		res :=
			p.y.i == pY.i &&
			p.x.i >= min_fixed(l0.x, l1.x).i &&
			p.x.i <= max_fixed(l0.x, l1.x).i &&
			p.y.i >= min_fixed(l0.y, l1.y).i &&
			p.y.i <= max_fixed(l0.y, l1.y).i
		return res, div(sub(p.x, minX), sub(maxX, minX))
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
	when intrinsics.type_is_float(T) {
		a := v1.y - v0.y
		b := v0.x - v1.x
		c := v1.x * v0.y + v0.x * v1.y
		res := a * p.x + b * p.y + c
		return res == 0, res
	} else {
		when intrinsics.type_is_specialization_of(T, fixed.Fixed) {

			add :: fixed.add
			sub :: fixed.sub
			mul :: fixed.mul
		} else {
			add :: fixed_bcd.add
			sub :: fixed_bcd.sub
			mul :: fixed_bcd.mul
		}

		a := sub(v1.y, v0.y)
		b := sub(v0.x, v1.x)
		c := add(mul(v1.x, v0.y), mul(v0.x, v1.y))
		res := add(add(mul(a, p.x), mul(b, p.y)), c)
		return res.i == 0, res
	}
}

PointLineLeftOrRight :: #force_inline proc "contextless" (
	p: [2]$T,
	l0: [2]T,
	l1: [2]T,
) -> T where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	when intrinsics.type_is_float(T) {
		return (l1.x - l0.x) * (p.y - l0.y) - (p.x - l0.x) * (l1.y - l0.y)
	} else when intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
		return fixed_bcd.sub(
			fixed_bcd.mul(fixed_bcd.sub(l1.x, l0.x), fixed_bcd.sub(p.y, l0.y)),
			fixed_bcd.mul(fixed_bcd.sub(p.x, l0.x), fixed_bcd.sub(l1.y, l0.y)),
		)
	} else {
		return fixed.sub(
			fixed.mul(fixed.sub(l1.x, l0.x), fixed.sub(p.y, l0.y)),
			fixed.mul(fixed.sub(p.x, l0.x), fixed.sub(l1.y, l0.y)),
		)
	}
}

PointInPolygon :: proc "contextless" (
	p: [2]$T,
	polygon: [][2]T,
) -> PointInPolygonResult where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	when intrinsics.type_is_float(T) {
		isPointOnSegment :: proc "contextless" (p: [2]$T, p1: [2]T, p2: [2]T) -> bool {
			return(
				CrossProductSign(p1, p2, p) == 0 &&
				p.x >= min(p1.x, p2.x) &&
				p.x <= max(p1.x, p2.x) &&
				p.y >= min(p1.y, p2.y) &&
				p.y <= max(p1.y, p2.y) \
			)
		}
		windingNumber := 0
		for i in 0 ..< len(polygon) {
			p1 := polygon[i]
			p2 := polygon[(i + 1) % len(polygon)]
			if isPointOnSegment(p, p1, p2) do return .On

			if p1.y <= p.y {
				if p2.y > p.y && CrossProductSign(p1, p2, p) > 0 do windingNumber += 1
			} else {
				if p2.y <= p.y && CrossProductSign(p1, p2, p) < 0 do windingNumber -= 1
			}
		}
		return windingNumber != 0 ? .Inside : .Outside
	} else when intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
		isPointOnSegment :: proc "contextless" (p: [2]$T, p1: [2]T, p2: [2]T) -> bool {
			return(
				CrossProductSign(p1, p2, p) == 0 &&
				p.x.i >= min_fixed(p1.x, p2.x).i &&
				p.x.i <= max_fixed(p1.x, p2.x).i &&
				p.y.i >= min_fixed(p1.y, p2.y).i &&
				p.y.i <= max_fixed(p1.y, p2.y).i \
			)
		}
		windingNumber := 0
		for i in 0 ..< len(polygon) {
			p1 := polygon[i]
			p2 := polygon[(i + 1) % len(polygon)]
			if isPointOnSegment(p, p1, p2) do return .On

			if p1.y.i <= p.y.i {
				if p2.y.i > p.y.i && CrossProductSign(p1, p2, p) > 0 do windingNumber += 1
			} else {
				if p2.y.i <= p.y.i && CrossProductSign(p1, p2, p) < 0 do windingNumber -= 1
			}
		}
		return windingNumber != 0 ? .Inside : .Outside
	} else {
		isPointOnSegment :: proc "contextless" (p: [2]$T, p1: [2]T, p2: [2]T) -> bool {
			return(
				CrossProductSign(p1, p2, p) == 0 &&
				p.x.i >= min_fixed(p1.x, p2.x).i &&
				p.x.i <= max_fixed(p1.x, p2.x).i &&
				p.y.i >= min_fixed(p1.y, p2.y).i &&
				p.y.i <= max_fixed(p1.y, p2.y).i \
			)
		}
		windingNumber := 0
		for i in 0 ..< len(polygon) {
			p1 := polygon[i]
			p2 := polygon[(i + 1) % len(polygon)]
			if isPointOnSegment(p, p1, p2) do return .On

			if p1.y.i <= p.y.i {
				if p2.y.i > p.y.i && CrossProductSign(p1, p2, p) > 0 do windingNumber += 1
			} else {
				if p2.y.i <= p.y.i && CrossProductSign(p1, p2, p) < 0 do windingNumber -= 1
			}
		}
		return windingNumber != 0 ? .Inside : .Outside
	}
}

vector_cross2_fixed :: #force_inline proc "contextless" (
	a, b: [2]$E,
) -> E where intrinsics.type_is_specialization_of(E, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(E, fixed_bcd.BCD) {
	when intrinsics.type_is_specialization_of(T, fixed.Fixed) {
		sub :: fixed.sub
		mul :: fixed.mul
	} else {
		sub :: fixed_bcd.sub
		mul :: fixed_bcd.mul
	}
	return sub(mul(a.x, b.y), mul(a.y, b.x))
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
		when intrinsics.type_is_specialization_of(T, fixed.Fixed) {

			add :: fixed.add
			sub :: fixed.sub
			mul :: fixed.mul
			three_T :: T {
				i = 3 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
			}
		} else {
			add :: fixed_bcd.add
			sub :: fixed_bcd.sub
			mul :: fixed_bcd.mul
			three_T :: T {
				i = 3 * fixed_bcd._SCALE_TABLE[intrinsics.type_polymorphic_record_parameter_value(T, 0) - 1],
			}
		}

		area: T = {}
		p: [2]T = {}
		for i in 0 ..< len(polygon) {
			j := (i + 1) % len(polygon)
			factor := vector_cross2_fixed(polygon[i], polygon[j])
			area = add(area, factor)
			sum_pt := [2]T{add(polygon[i].x, polygon[j].x), add(polygon[i].y, polygon[j].y)}
			p.x = add(p.x, mul(sum_pt.x, factor))
			p.y = add(p.y, mul(sum_pt.y, factor))
		}
		area = mul(area, three_T)
		p.x = div(p.x, area)
		p.y = div(p.y, area)
		return p
	}
}

GetPolygonOrientation :: proc "contextless" (
	polygon: [][2]$T,
) -> PolyOrientation where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	when intrinsics.type_is_float(T) {
		res: T = 0
		for i in 0 ..< len(polygon) {
			j := (i + 1) % len(polygon)
			factor := (polygon[j].x - polygon[i].x) * (polygon[j].y + polygon[i].y)
			res += factor
		}
		return res > 0 ? .Clockwise : .CounterClockwise
	} else {
		res: T = {}
		for i in 0 ..< len(polygon) {
			j := (i + 1) % len(polygon)
			when intrinsics.type_is_specialization_of(T, fixed.Fixed) {
				factor :=
					(polygon[j].x.i - polygon[i].x.i) *
					(polygon[j].y.i + polygon[i].y.i) >>
					intrinsics.type_polymorphic_record_parameter_value(T, 1)
			} else {
				factor :=
					(polygon[j].x.i - polygon[i].x.i) *
					(polygon[j].y.i + polygon[i].y.i) /
					fixed_bcd._SCALE_TABLE[intrinsics.type_polymorphic_record_parameter_value(T, 0) - 1]
			}
			res.i += factor
		}
		return res.i > 0 ? .Clockwise : .CounterClockwise
	}
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
	when intrinsics.type_is_float(T) {
		if check_is_touching {
			if a1 == b1 || a1 == b2 do return .none, a1
			else if a2 == b1 || a2 == b2 do return .none, a2
		}

		den: T = (b2.y - b1.y) * (a2.x - a1.x) - (b2.x - b1.x) * (a2.y - a1.y)
		if den == 0.0 {
			return .collinear, {}
		}

		ua := ((b2.x - b1.x) * (a1.y - b1.y) - (b2.y - b1.y) * (a1.x - b1.x))
		ub := ((a2.x - a1.x) * (a1.y - b1.y) - (a2.y - a1.y) * (a1.x - b1.x))

		res_den :=
			den > 0.0 ? (ua >= 0.0 && ub >= 0.0 && ua <= den && ub <= den) : (ua >= den && ub >= den && ua <= 0.0 && ub <= 0.0)

		t := ua / den
		return res_den ? .intersect : .none, [2]T{a1.x + t * (a2.x - a1.x), a1.y + t * (a2.y - a1.y)}
	} else {
		when intrinsics.type_is_specialization_of(T, fixed.Fixed) {

			add :: fixed.add
			sub :: fixed.sub
			mul :: fixed.mul
			equal :: fixed_ex.equal
		} else {
			add :: fixed_bcd.add
			sub :: fixed_bcd.sub
			mul :: fixed_bcd.mul
			equal :: fixed_bcd.equal
		}

		if check_is_touching {
			if equal(a1, b1) || equal(a1, b2) do return .none, a1
			else if equal(a2, b1) || equal(a2, b2) do return .none, a2
		}

		den: T = sub(mul(sub(b2.y, b1.y), sub(a2.x, a1.x)), mul(sub(b2.x, b1.x), sub(a2.y, a1.y)))
		if den.i == 0 {
			return .collinear, {}
		}

		ua := sub(mul(sub(b2.x, b1.x), sub(a1.y, b1.y)), mul(sub(b2.y, b1.y), sub(a1.x, b1.x)))
		ub := sub(mul(sub(a2.x, a1.x), sub(a1.y, b1.y)), mul(sub(a2.y, a1.y), sub(a1.x, b1.x)))

		res_den :=
			den.i > 0 ? (ua.i >= 0 && ub.i >= 0 && ua.i <= den.i && ub.i <= den.i) : (ua.i >= den.i && ub.i >= den.i && ua.i <= 0 && ub.i <= 0)

		t := fixed_bcd.div(ua, den)
		return res_den ? .intersect : .none, [2]T{add(a1.x, mul(t, sub(a2.x, a1.x))), add(a1.y, mul(t, sub(a2.y, a1.y)))}
	}
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
	when intrinsics.type_is_float(T) {
		if check_is_touching && a1 == b1 || a2 == b1 || a1 == b2 || a2 == b2 do return .none

		den: T = (b2.y - b1.y) * (a2.x - a1.x) - (b2.x - b1.x) * (a2.y - a1.y)
		if den == 0.0 {
			return .collinear
		}
		ua := ((b2.x - b1.x) * (a1.y - b1.y) - (b2.y - b1.y) * (a1.x - b1.x))
		ub := ((a2.x - a1.x) * (a1.y - b1.y) - (a2.y - a1.y) * (a1.x - b1.x))
		res_den :=
			den > 0.0 ? (ua >= 0.0 && ub >= 0.0 && ua <= den && ub <= den) : (ua >= den && ub >= den && ua <= 0.0 && ub <= 0.0)
		return res_den ? .intersect : .none
	} else {
		when intrinsics.type_is_specialization_of(T, fixed.Fixed) {

			add :: fixed.add
			sub :: fixed.sub
			mul :: fixed.mul
			equal :: fixed_ex.equal
		} else {
			add :: fixed_bcd.add
			sub :: fixed_bcd.sub
			mul :: fixed_bcd.mul
			equal :: fixed_bcd.equal
		}

		if check_is_touching && equal(a1, b1) || equal(a2, b1) || equal(a1, b2) || equal(a2, b2) do return .none

		den: T = sub(mul(sub(b2.y, b1.y), sub(a2.x, a1.x)), mul(sub(b2.x, b1.x), sub(a2.y, a1.y)))
		if den.i == 0 do return .collinear
		ua := sub(mul(sub(b2.x, b1.x), sub(a1.y, b1.y)), mul(sub(b2.y, b1.y), sub(a1.x, b1.x)))
		ub := sub(mul(sub(a2.x, a1.x), sub(a1.y, b1.y)), mul(sub(a2.y, a1.y), sub(a1.x, b1.x)))
		res_den :=
			den.i > 0 ? (ua.i >= 0 && ub.i >= 0 && ua.i <= den.i && ub.i <= den.i) : (ua.i >= den.i && ub.i >= den.i && ua.i <= 0 && ub.i <= 0)
		return res_den ? .intersect : .none
	}
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
		when intrinsics.type_is_specialization_of(T, fixed.Fixed) {

			sub :: fixed.sub
			mul :: fixed.mul
			div :: fixed.div
		} else {
			sub :: fixed_bcd.sub
			mul :: fixed_bcd.mul
			div :: fixed_bcd.div
		}

		AB := sub(l1, l0)
		AC := sub(p, l0)

		return fixed.add(l0, mul(AB, div(vector_dot_fixed(AB, AC), vector_dot_fixed(AB, AB))))
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
	when intrinsics.type_is_float(T) {
		dx := l2.x - l1.x
		dy := l2.y - l1.y
		ax := pt.x - l1.x
		ay := pt.y - l1.y
		qNum := ax * dx + ay * dy
		denom := dx * dx + dy * dy
		if qNum < 0 {
			return ax * ax + ay * ay, 1.0
		} else if qNum > denom {
			bx := pt.x - l2.x
			by := pt.y - l2.y
			return bx * bx + by * by, 1.0
		} else {
			cross := ax * dy - dx * ay
			return cross * cross, denom
		}
	} else {
		when intrinsics.type_is_specialization_of(T, fixed.Fixed) {

			add :: fixed.add
			sub :: fixed.sub
			mul :: fixed.mul
			one_T :: T {
				i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
			}
		} else {
			add :: fixed_bcd.add
			sub :: fixed_bcd.sub
			mul :: fixed_bcd.mul
			one_T := fixed_bcd.init_const(
				1,
				0,
				0,
				intrinsics.type_polymorphic_record_parameter_value(T, 0),
			)
		}

		dx := sub(l2.x, l1.x)
		dy := sub(l2.y, l1.y)
		ax := sub(pt.x, l1.x)
		ay := sub(pt.y, l1.y)
		qNum := add(mul(ax, dx), mul(ay, dy))
		denom := add(mul(dx, dx), mul(dy, dy))

		if qNum.i < 0 {
			return add(mul(ax, ax), mul(ay, ay)), one_T
		} else if qNum.i > denom.i {
			bx := sub(pt.x, l2.x)
			by := sub(pt.y, l2.y)
			return add(mul(bx, bx), mul(by, by)), one_T
		} else {
			cross := sub(mul(ax, dy), mul(dx, ay))
			return mul(cross, cross), denom
		}
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

@(require_results)
vector_dot_fixed :: proc "contextless" (
	a, b: $T/[$N]$E,
) -> (
	c: E,
) where intrinsics.type_is_specialization_of(E, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) #no_bounds_check {
	when intrinsics.type_is_specialization_of(T, fixed.Fixed) {
		context = runtime.Context {
			allocator      = runtime.panic_allocator(),
			temp_allocator = runtime.panic_allocator(),
		} //dummy
		add :: fixed.add
		mul :: fixed.mul
	} else {
		add :: fixed_bcd.add
		mul :: fixed_bcd.mul
	}
	#unroll for i in 0 ..< N {
		c = fixed.add(c, fixed.mul(a[i], b[i]))
	}
	return
}


CvtQuadraticToCubic0 :: #force_inline proc "contextless" (
	_start: linalg.Vector2f32,
	_control: linalg.Vector2f32,
) -> linalg.Vector2f32 {
	return linalg.Vector2f32 {
		_start.x + (2.0 / 3.0) * (_control.x - _start.x),
		_start.y + (2.0 / 3.0) * (_control.y - _start.y),
	}
}
CvtQuadraticToCubic1 :: #force_inline proc "contextless" (
	_end: linalg.Vector2f32,
	_control: linalg.Vector2f32,
) -> linalg.Vector2f32 {
	return CvtQuadraticToCubic0(_end, _control)
}

rect_line_init :: proc "contextless" (_rect: Rectf32) -> [4]linalg.Vector2f32 {
	return [4]linalg.Vector2f32 {
		linalg.Vector2f32{_rect.left, _rect.top},
		linalg.Vector2f32{_rect.left, _rect.bottom},
		linalg.Vector2f32{_rect.right, _rect.bottom},
		linalg.Vector2f32{_rect.right, _rect.top},
	}
}

round_rect_line_init :: proc "contextless" (
	_rect: Rectf32,
	_radius: f32,
) -> (
	pts: [16]linalg.Vector2f32,
	is_curves: [16]bool,
) {
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

	return [16]linalg.Vector2f32 {
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
	}, [16]bool {
		false,
		true,
		true,
		false,
		false,
		true,
		true,
		false,
		false,
		true,
		true,
		false,
		false,
		true,
		true,
		false,
	}
}

circle_cubic_init :: proc "contextless" (
	_center: linalg.Vector2f32,
	_r: f32,
) -> (
	pts: [12]linalg.Vector2f32,
	is_curves: [12]bool,
) {
	t: f32 = (4.0 / 3.0) * math.tan_f32(math.PI / 8.0)
	tt := t * _r
	return [12]linalg.Vector2f32 {
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
	}, [12]bool{false, true, true, false, true, true, false, true, true, false, true, true}
}

ellipse_cubic_init :: proc "contextless" (
	_center: linalg.Vector2f32,
	_rxy: linalg.Vector2f32,
) -> (
	pts: [12]linalg.Vector2f32,
	is_curves: [12]bool,
) {
	t: f32 = (4.0 / 3.0) * math.tan_f32(math.PI / 8.0)
	ttx := t * _rxy.x
	tty := t * _rxy.y
	return [12]linalg.Vector2f32 {
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
	}, [12]bool{false, true, true, false, true, true, false, true, true, false, true, true}
}

//(p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x) == (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x)
@(require_results)
CrossProductSign :: proc "contextless" (
	p1, p2, p3: [2]$T,
) -> int where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	when intrinsics.type_is_float(T) {
		a := p2.x - p1.x
		b := p3.y - p2.y
		c := p2.y - p1.y
		d := p3.x - p2.x
		ab := a * b
		cd := c * d
		if ab > cd do return 1
		if ab < cd do return -1
	} else {
		when intrinsics.type_is_specialization_of(T, fixed.Fixed) {

			add :: fixed.add
			sub :: fixed.sub
			mul :: fixed.mul
		} else {
			add :: fixed_bcd.add
			sub :: fixed_bcd.sub
			mul :: fixed_bcd.mul
		}

		a := sub(p2.x, p1.x)
		b := sub(p3.y, p2.y)
		c := sub(p2.y, p1.y)
		d := sub(p3.x, p2.x)

		when intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
			return fixed_bcd.compare_product(a, b, c, d)
		} else {
			ab := mul(a, b)
			cd := mul(c, d)
			if ab.i > cd.i do return 1
			else if ab.i < cd.i do return -1
		}
	}
	return 0
}

DotProduct :: proc "contextless" (
	p1, p2, p3: [2]$T,
) -> T where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	when intrinsics.type_is_float(T) {
		a := p2.x - p1.x
		b := p3.x - p2.x
		c := p2.y - p1.y
		d := p3.y - p2.y
		return a * b + c * d
	} else {
		when intrinsics.type_is_specialization_of(T, fixed.Fixed) {

			add :: fixed.add
			sub :: fixed.sub
			mul :: fixed.mul
		} else {
			add :: fixed_bcd.add
			sub :: fixed_bcd.sub
			mul :: fixed_bcd.mul
		}

		a := sub(p2.x, p1.x)
		b := sub(p3.x, p2.x)
		c := sub(p2.y, p1.y)
		d := sub(p3.y, p2.y)
		return add(mul(a, b), mul(c, d))
	}
}

// InCircleTest: returns determinant. Positive => D inside circle through A,B,C (when ABC is CCW).
InCircleTest :: proc "contextless" (
	ptA, ptB, ptC, ptD: [2]$T,
) -> T where intrinsics.type_is_float(T) ||
	intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) {
	when intrinsics.type_is_float(T) {
		m00 := ptA.x - ptD.x
		m01 := ptA.y - ptD.y
		m02 := m00 * m00 + m01 * m01
		m10 := ptB.x - ptD.x
		m11 := ptB.y - ptD.y
		m12 := m10 * m10 + m11 * m11
		m20 := ptC.x - ptD.x
		m21 := ptC.y - ptD.y
		m22 := m20 * m20 + m21 * m21
		return(
			m00 * (m11 * m22 - m21 * m12) -
			m10 * (m01 * m22 - m21 * m02) +
			m20 * (m01 * m12 - m11 * m02) \
		)
	} else {
		when intrinsics.type_is_specialization_of(T, fixed.Fixed) {

			add :: fixed.add
			sub :: fixed.sub
			mul :: fixed.mul
		} else {
			add :: fixed_bcd.add
			sub :: fixed_bcd.sub
			mul :: fixed_bcd.mul
		}

		m00 := sub(ptA.x, ptD.x)
		m01 := sub(ptA.y, ptD.y)
		m02 := add(mul(m00, m00), mul(m01, m01))
		m10 := sub(ptB.x, ptD.x)
		m11 := sub(ptB.y, ptD.y)
		m12 := add(mul(m10, m10), mul(m11, m11))
		m20 := sub(ptC.x, ptD.x)
		m21 := sub(ptC.y, ptD.y)
		m22 := add(mul(m20, m20), mul(m21, m21))
		return add(
			sub(
				mul(m00, sub(mul(m11, m22), mul(m21, m12))),
				mul(m10, sub(mul(m01, m22), mul(m21, m02))),
			),
			mul(m20, sub(mul(m01, m12), mul(m11, m02))),
		)
	}
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

