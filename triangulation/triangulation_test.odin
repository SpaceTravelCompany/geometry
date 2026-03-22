package triangulation

import "core:math"
import "core:testing"
import "shared:utils_private"
import "shared:utils_private/fixed_bcd"

@(private = "file")
DEF_FRAC_DIGITS :: fixed_bcd.MAX_FRAC_DIGITS

@(private)
_make_square_path_bcd :: proc(
	x0, y0, size: fixed_bcd.BCD($FRAC_DIGITS),
	invent := false,
	allocator := context.allocator,
) -> [][2]fixed_bcd.BCD(FRAC_DIGITS) {
	result := utils_private.make_non_zeroed_slice([][2]fixed_bcd.BCD(FRAC_DIGITS), 4, allocator)
	two := fixed_bcd.init_const(2, 0, 0, FRAC_DIGITS)
	half := fixed_bcd.div(size, two)
	if !invent {
		result[0] = {fixed_bcd.add(x0, half), fixed_bcd.add(y0, half)}
		result[1] = {fixed_bcd.sub(x0, half), fixed_bcd.add(y0, half)}
		result[2] = {fixed_bcd.sub(x0, half), fixed_bcd.sub(y0, half)}
		result[3] = {fixed_bcd.add(x0, half), fixed_bcd.sub(y0, half)}
	} else {
		result[0] = {fixed_bcd.sub(x0, half), fixed_bcd.sub(y0, half)}
		result[1] = {fixed_bcd.add(x0, half), fixed_bcd.sub(y0, half)}
		result[2] = {fixed_bcd.add(x0, half), fixed_bcd.add(y0, half)}
		result[3] = {fixed_bcd.sub(x0, half), fixed_bcd.add(y0, half)}
	}
	return result
}


@(test)
test_triangulation_2square :: proc(t: ^testing.T) {
	x0 := fixed_bcd.init_const(2, 0, 0, DEF_FRAC_DIGITS)
	y0 := fixed_bcd.init_const(0, 0, 0, DEF_FRAC_DIGITS)
	size := fixed_bcd.init_const(100, 0, 0, DEF_FRAC_DIGITS)

	x1 := fixed_bcd.init_const(25, 0, 0, DEF_FRAC_DIGITS)
	y1 := fixed_bcd.init_const(-25, 0, 0, DEF_FRAC_DIGITS)
	size2 := fixed_bcd.init_const(50, 0, 0, DEF_FRAC_DIGITS)

	square := _make_square_path_bcd(x0, y0, size)
	defer delete(square)

	square2 := _make_square_path_bcd(x1, y1, size2)
	defer delete(square2)

	poly := [][][2]fixed_bcd.BCD(DEF_FRAC_DIGITS){square, square2}
	indices, err := TrianguatePolygons_Fixed(poly)
	defer delete(indices)

	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(indices), 12)
}

// @(test)
// test_cross_product_sign_convention :: proc(t: ^testing.T) {
// 	p0 := [2]fixed_bcd.BCD(DEF_FRAC_DIGITS) {
// 		fixed_bcd.init_const(0, 0, 0, DEF_FRAC_DIGITS),
// 		fixed_bcd.init_const(0, 0, 0, DEF_FRAC_DIGITS),
// 	}
// 	p1 := [2]fixed_bcd.BCD(DEF_FRAC_DIGITS) {
// 		fixed_bcd.init_const(1, 0, 0, DEF_FRAC_DIGITS),
// 		fixed_bcd.init_const(0, 0, 0, DEF_FRAC_DIGITS),
// 	}
// 	p2 := [2]fixed_bcd.BCD(DEF_FRAC_DIGITS) {
// 		fixed_bcd.init_const(1, 0, 0, DEF_FRAC_DIGITS),
// 		fixed_bcd.init_const(1, 0, 0, DEF_FRAC_DIGITS),
// 	}

// 	testing.expect_value(t, CrossProductSign(p0, p1, p2), 1)
// 	testing.expect_value(
// 		t,
// 		GetPolygonOrientation([][2]fixed_bcd.BCD(DEF_FRAC_DIGITS){p0, p1, p2}),
// 		PolyOrientation.CounterClockwise,
// 	)
// 	testing.expect_value(t, CrossProductSign(p0, p2, p1), -1)
// 	testing.expect_value(
// 		t,
// 		GetPolygonOrientation([][2]fixed_bcd.BCD(DEF_FRAC_DIGITS){p0, p2, p1}),
// 		PolyOrientation.Clockwise,
// 	)
// }
