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


poly1 := [?][2]f64 {
	{659095934, 582732471},
	{688229859, 641580231},
	{790136932, 742292480},
	{734002697, 717320583},
	{568341100, 767739453},
	{514770968, 587168730},
	{520332261, 821050012},
	{498064340, 703595158},
	{379746222, 688455292},
	{295113410, 567063295},
	{220890815, 495587766},
	{251213275, 481484985},
	{380843262, 489802436},
	{259130906, 425382705},
	{185884836, 383423393},
	{234250146, 399675871},
	{288650298, 372266589},
	{184608388, 267719266},
	{373473588, 389571306},
	{404368012, 298780630},
	{343564255, 137385770},
	{470369848, 258889217},
	{501650983, 243889661},
	{510066029, 411945600},
	{532543072, 347086099},
	{542055441, 383498744},
	{538240888, 411267759},
	{630160722, 208121688},
	{743191330, 206034390},
	{603732120, 450642775},
	{789265889, 463295358},
	{739813150, 499481850},
}

@(test)
test_custom :: proc(t: ^testing.T) {
	poly2: [][2]fixed_bcd.BCD(DEF_FRAC_DIGITS) = utils_private.make_non_zeroed_slice(
		[][2]fixed_bcd.BCD(DEF_FRAC_DIGITS),
		len(poly1),
		context.allocator,
	)
	defer {
		delete(poly2)
	}

	for i in 0 ..< len(poly1) {
		poly2[i][0] = fixed_bcd.from_f64(DEF_FRAC_DIGITS, poly1[i][0] / 10000.0)
		poly2[i][1] = fixed_bcd.from_f64(DEF_FRAC_DIGITS, poly1[i][1] / 10000.0)
	}

	// res, res_open, err := clipper.BooleanOp_Fixed(
	// 	.Union,
	// 	DEF_FRAC_DIGITS,
	// 	[][][2]fixed_bcd.BCD(DEF_FRAC_DIGITS){poly2[:]},
	// 	nil,
	// 	nil,
	// 	.NonZero,
	// )
	// defer {
	// 	for r in res {
	// 		delete(r)
	// 	}
	// 	delete(res)
	// }

	indices, terr := TrianguatePolygons_Fixed([][][2]fixed_bcd.BCD(DEF_FRAC_DIGITS){poly2})
	defer delete(indices)
}
