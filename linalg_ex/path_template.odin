package linalg_ex

import "base:intrinsics"
import "core:math"
import "core:math/fixed"


CircleCubicInit :: proc "contextless" (
	_center: [2]$T,
	_r: T,
) -> (
	pts: [12][2]T,
	is_curves: [12]bool,
) where intrinsics.type_is_float(T) {
	t :: 0.55228474983079332144
	tt := t * _r
	cx := _center[0]
	cy := _center[1]
	return [12][2]T {
		{cx - _r, cy},
		{cx - _r, cy - tt},
		{cx - tt, cy - _r},
		{cx, cy - _r},
		{cx + tt, cy - _r},
		{cx + _r, cy - tt},
		{cx + _r, cy},
		{cx + _r, cy + tt},
		{cx + tt, cy + _r},
		{cx, cy + _r},
		{cx - tt, cy + _r},
		{cx - _r, cy + tt},
	}, [12]bool{false, true, true, false, true, true, false, true, true, false, true, true}
}

CircleCubicInit_Fixed :: proc "contextless" (
	_center: [2]$T/fixed.Fixed($Backing, $Fraction_Width),
	_r: T,
) -> (
	pts: [12][2]T,
	is_curves: [12]bool,
) {
	t: T
	fixed.init_from_parts(
		&t,
		0,
		Backing(math.floor(f64(1 << Fraction_Width) * 0.55228474983079332144)),
	)
	tt := fixed.mul(t, _r)
	cx := _center.x
	cy := _center.y
	return [12][2]T {
		{fixed.sub(cx, _r), cy},
		{fixed.sub(cx, _r), fixed.sub(cy, tt)},
		{fixed.sub(cx, tt), fixed.sub(cy, _r)},
		{cx, fixed.sub(cy, _r)},
		{fixed.add(cx, tt), fixed.sub(cy, _r)},
		{fixed.add(cx, _r), fixed.sub(cy, tt)},
		{fixed.add(cx, _r), cy},
		{fixed.add(cx, _r), fixed.add(cy, tt)},
		{fixed.add(cx, tt), fixed.add(cy, _r)},
		{cx, fixed.add(cy, _r)},
		{fixed.sub(cx, tt), fixed.add(cy, _r)},
		{fixed.sub(cx, _r), fixed.add(cy, tt)},
	}, [12]bool{false, true, true, false, true, true, false, true, true, false, true, true}
}

RectLineInit :: proc "contextless" (
	_rect: Rect_($T),
) -> [4][2]T where intrinsics.type_is_float(T) {
	return [4][2]T {
		{_rect.left, _rect.top},
		{_rect.left, _rect.bottom},
		{_rect.right, _rect.bottom},
		{_rect.right, _rect.top},
	}
}

RectLineInit_Fixed :: proc "contextless" (
	_rect: Rect_($T/fixed.Fixed($Backing, $Fraction_Width)),
) -> [4][2]T {
	return [4][2]T {
		{_rect.left, _rect.top},
		{_rect.left, _rect.bottom},
		{_rect.right, _rect.bottom},
		{_rect.right, _rect.top},
	}
}

RoundRectLineInit :: proc "contextless" (
	_rect: Rect_($T),
	_radius: T,
) -> (
	pts: [16][2]T,
	is_curves: [16]bool,
) where intrinsics.type_is_float(T) {
	r := _radius

	half_width := (_rect.right - _rect.left) / 2.0
	dh := _rect.bottom - _rect.top
	if dh < 0 do dh = -dh
	half_height := dh * half
	if r > half_width do r = half_width
	if r > half_height do r = half_height

	t :: 0.55228474983079332144
	tt := t * r

	L := _rect.left
	R := _rect.right
	rt := _rect.top
	rb := _rect.bottom

	return [16][2]T {
		// Top-left corner (cubic) - counter-clockwise: from top to left
		// Note: y increases upward, _rect.top is top (larger y), _rect.bottom is bottom (smaller y)
		{L + r, rt},
		{L + r - tt, rt},
		{L, rt - r + tt},
		// Left line - counter-clockwise: from top to bottom (y decreases)
		{L, rt - r},
		// Bottom-left corner (cubic) - counter-clockwise: from left to bottom
		{L, rb + r},
		{L, rb + r - tt},
		{L + r - tt, rb},
		// Bottom line - counter-clockwise: from left to right
		{L + r, rb},
		// Bottom-right corner (cubic) - counter-clockwise: from bottom to right
		{R - r, rb},
		{R - r + tt, rb},
		{R, rb + r - tt},
		// Right line - counter-clockwise: from bottom to top (y increases)
		{R, rb + r},
		// Top-right corner (cubic) - counter-clockwise: from right to top
		{R, rt - r},
		{R, rt - r + tt},
		{R - r + tt, rt},
		// Top line - counter-clockwise: from right to left
		{R - r, rt},
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

RoundRectLineInit_Fixed :: proc "contextless" (
	_rect: Rect_($F/fixed.Fixed($Backing, $Fraction_Width)),
	_radius: F,
) -> (
	pts: [16][2]F,
	is_curves: [16]bool,
) {
	two: F : F{i = 2 << Fraction_Width}

	r := _radius
	half_width := fixed.div(fixed.sub(_rect.right, _rect.left), two)
	dh := fixed.sub(_rect.bottom, _rect.top)
	if dh.i < 0 do dh = F {
		i = -dh.i,
	}
	half_height := fixed.div(dh, two)
	if fixed.sub(half_width, r).i < 0 do r = half_width
	if fixed.sub(half_height, r).i < 0 do r = half_height

	t: F
	fixed.init_from_parts(
		&t,
		0,
		Backing(math.floor(f64(1 << Fraction_Width) * 0.55228474983079332144)),
	)
	tt := fixed.mul(t, r)

	L := _rect.left
	R := _rect.right
	T := _rect.top
	B := _rect.bottom

	return [16][2]F {
		{fixed.add(L, r), T},
		{fixed.sub(fixed.add(L, r), tt), T},
		{L, fixed.add(fixed.sub(T, r), tt)},
		{L, fixed.sub(T, r)},
		{L, fixed.add(B, r)},
		{L, fixed.sub(fixed.add(B, r), tt)},
		{fixed.sub(fixed.add(L, r), tt), B},
		{fixed.add(L, r), B},
		{fixed.sub(R, r), B},
		{fixed.add(fixed.sub(R, r), tt), B},
		{R, fixed.sub(fixed.add(B, r), tt)},
		{R, fixed.add(B, r)},
		{R, fixed.sub(T, r)},
		{R, fixed.add(fixed.sub(T, r), tt)},
		{fixed.add(fixed.sub(R, r), tt), T},
		{fixed.sub(R, r), T},
	}, [16]bool{false, true, true, false, false, true, true, false, false, true, true, false, false, true, true, false}
}


EllipseCubicInit :: proc "contextless" (
	_center: [2]$T,
	_rxy: [2]T,
) -> (
	pts: [12][2]T,
	is_curves: [12]bool,
) where intrinsics.type_is_float(T) {
	t :: 0.55228474983079332144

	ttx := t * _rxy[0]
	tty := t * _rxy[1]
	cx := _center[0]
	cy := _center[1]
	rx := _rxy[0]
	ry := _rxy[1]
	return [12][2]T {
		{cx - rx, cy},
		{cx - rx, cy - tty},
		{cx - ttx, cy - ry},
		{cx, cy - ry},
		{cx + ttx, cy - ry},
		{cx + rx, cy - tty},
		{cx + rx, cy},
		{cx + rx, cy + tty},
		{cx + ttx, cy + ry},
		{cx, cy + ry},
		{cx - ttx, cy + ry},
		{cx - rx, cy + tty},
	}, [12]bool{false, true, true, false, true, true, false, true, true, false, true, true}
}

EllipseCubicInit_Fixed :: proc "contextless" (
	_center: [2]$T/fixed.Fixed($Backing, $Fraction_Width),
	_rxy: [2]T,
) -> (
	pts: [12][2]T,
	is_curves: [12]bool,
) {
	t: T
	fixed.init_from_parts(
		&t,
		0,
		Backing(math.floor(f64(1 << Fraction_Width) * 0.55228474983079332144)),
	)

	ttx := fixed.mul(t, _rxy.x)
	tty := fixed.mul(t, _rxy.y)
	cx := _center.x
	cy := _center.y
	rx := _rxy.x
	ry := _rxy.y
	return [12][2]T {
		{fixed.sub(cx, rx), cy},
		{fixed.sub(cx, rx), fixed.sub(cy, tty)},
		{fixed.sub(cx, ttx), fixed.sub(cy, ry)},
		{cx, fixed.sub(cy, ry)},
		{fixed.add(cx, ttx), fixed.sub(cy, ry)},
		{fixed.add(cx, rx), fixed.sub(cy, tty)},
		{fixed.add(cx, rx), cy},
		{fixed.add(cx, rx), fixed.add(cy, tty)},
		{fixed.add(cx, ttx), fixed.add(cy, ry)},
		{cx, fixed.add(cy, ry)},
		{fixed.sub(cx, ttx), fixed.add(cy, ry)},
		{fixed.sub(cx, rx), fixed.add(cy, tty)},
	}, [12]bool{false, true, true, false, true, true, false, true, true, false, true, true}
}

