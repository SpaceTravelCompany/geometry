package linalg_ex

import "base:intrinsics"

CircleCubicInit :: proc "contextless" (
	_center: [2]$T,
	_r: T,
) -> (
	pts: [12][2]T,
	isCurves: [12]bool,
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

RoundRectLineInit :: proc "contextless" (
	_rect: Rect_($T),
	_radius: T,
) -> (
	pts: [16][2]T,
	isCurves: [16]bool,
) where intrinsics.type_is_float(T) {
	r := _radius

	halfWidth := (_rect.right - _rect.left) / 2.0
	dh := _rect.bottom - _rect.top
	if dh < 0 do dh = -dh
	halfHeight := dh * halfWidth
	if r > halfWidth do r = halfWidth
	if r > halfHeight do r = halfHeight

	t :: 0.55228474983079332144
	tt := t * r

	L := _rect.left
	R := _rect.right
	rt := _rect.top
	rb := _rect.bottom

	return [16][2]T {
		{L + r, rt},
		{L + r - tt, rt},
		{L, rt - r + tt},
		{L, rt - r},
		{L, rb + r},
		{L, rb + r - tt},
		{L + r - tt, rb},
		{L + r, rb},
		{R - r, rb},
		{R - r + tt, rb},
		{R, rb + r - tt},
		{R, rb + r},
		{R, rt - r},
		{R, rt - r + tt},
		{R - r + tt, rt},
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

EllipseCubicInit :: proc "contextless" (
	_center: [2]$T,
	_rxy: [2]T,
) -> (
	pts: [12][2]T,
	isCurves: [12]bool,
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
