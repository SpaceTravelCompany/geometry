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
	pts: [12][2]T,
	isCurves: [12]bool,
) where intrinsics.type_is_float(T) {
	r := _radius

	halfWidth := (_rect.right - _rect.left) / 2.0
	dh := _rect.bottom - _rect.top
	if dh < 0 do dh = -dh
	halfHeight := dh / 2.0
	if r > halfWidth do r = halfWidth
	if r > halfHeight do r = halfHeight

	L := _rect.left
	R := _rect.right
	rt := _rect.top
	rb := _rect.bottom

	// quadratic bezier로 quarter-circle 근사 (제어점 = 모서리 점)
	return [12][2]T {
		{L + r, rt}, // pt0:  top-left corner start (top edge)
		{L, rt}, // pt1:  control (corner point)
		{L, rt - r}, // pt2:  top-left corner end (left edge)
		{L, rb + r}, // pt3:  bottom-left corner start (left edge)
		{L, rb}, // pt4:  control (corner point)
		{L + r, rb}, // pt5:  bottom-left corner end (bottom edge)
		{R - r, rb}, // pt6:  bottom-right corner start (bottom edge)
		{R, rb}, // pt7:  control (corner point)
		{R, rb + r}, // pt8:  bottom-right corner end (right edge)
		{R, rt - r}, // pt9:  top-right corner start (right edge)
		{R, rt}, // pt10: control (corner point)
		{R - r, rt}, // pt11: top-right corner end (top edge)
	}, [12]bool {
		false, // pt0:  anchor
		true, // pt1:  control
		false, // pt2:  anchor
		false, // pt3:  anchor — line from pt2
		true, // pt4:  control
		false, // pt5:  anchor
		false, // pt6:  anchor — line from pt5
		true, // pt7:  control
		false, // pt8:  anchor
		false, // pt9:  anchor — line from pt8
		true, // pt10: control
		false, // pt11: anchor
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
