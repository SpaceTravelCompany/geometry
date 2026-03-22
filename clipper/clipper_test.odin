package clipper

import "base:intrinsics"
import "core:fmt"
import "core:testing"
import "shared:utils_private"
import "shared:utils_private/fixed_bcd"

@(private = "file")
DEF_FRAC_DIGITS :: 14

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

@(private)
_make_square_path_float :: proc(
	x0, y0, size: $T,
	invent := false,
	allocator := context.allocator,
) -> [][2]T where intrinsics.type_is_float(T) {
	result := utils_private.make_non_zeroed_slice([][2]T, 4, allocator)
	two :: T(2)
	half := T(size) / two
	if !invent {
		result[0] = {x0 + half, y0 + half}
		result[1] = {x0 - half, y0 + half}
		result[2] = {x0 - half, y0 - half}
		result[3] = {x0 + half, y0 - half}
	} else {
		result[0] = {x0 - half, y0 - half}
		result[1] = {x0 + half, y0 - half}
		result[2] = {x0 + half, y0 + half}
		result[3] = {x0 - half, y0 + half}
	}
	return result
}

@(test)
test_union_2square :: proc(t: ^testing.T) {
	x0 := fixed_bcd.init_const(0, 0, 0, DEF_FRAC_DIGITS)
	y0 := fixed_bcd.init_const(0, 0, 0, DEF_FRAC_DIGITS)
	size := fixed_bcd.init_const(100, 0, 0, DEF_FRAC_DIGITS)

	x1 := fixed_bcd.init_const(50, 0, 0, DEF_FRAC_DIGITS)
	y1 := fixed_bcd.init_const(-50, 0, 0, DEF_FRAC_DIGITS)
	size2 := fixed_bcd.init_const(100, 0, 0, DEF_FRAC_DIGITS)

	square := _make_square_path_bcd(x0, y0, size)
	defer delete(square)

	square2 := _make_square_path_bcd(x1, y1, size2)
	defer delete(square2)

	poly := [][][2]fixed_bcd.BCD(DEF_FRAC_DIGITS){square, square2}
	res, res_open, err := BooleanOp_Fixed(.Union, DEF_FRAC_DIGITS, poly, nil, nil)
	defer if res != nil {
		for r in res {
			delete(r)
		}
		delete(res)
	}
	defer if res_open != nil {
		for r in res_open {
			delete(r)
		}
		delete(res_open)
	}

	testing.expect_value(t, err, nil)
}

@(test)
test_union_2square_float :: proc(t: ^testing.T) {
	x0: f32 = 0.0
	y0: f32 = 0.0
	size: f32 = 100.0

	x1: f32 = 50.0
	y1: f32 = -50.0
	size2: f32 = 100.0

	square := _make_square_path_float(x0, y0, size)
	defer delete(square)

	square2 := _make_square_path_float(x1, y1, size2)
	defer delete(square2)

	poly := [][][2]f32{square, square2}
	res, res_open, err := BooleanOp(.Union, f32, poly, nil, nil)
	defer if res != nil {
		for r in res {
			delete(r)
		}
		delete(res)
	}
	defer if res_open != nil {
		for r in res_open {
			delete(r)
		}
		delete(res_open)
	}

	testing.expect_value(t, err, nil)
}

@(test)
test_intersection_2square :: proc(t: ^testing.T) {
	x0 := fixed_bcd.init_const(0, 0, 0, DEF_FRAC_DIGITS)
	y0 := fixed_bcd.init_const(0, 0, 0, DEF_FRAC_DIGITS)
	size := fixed_bcd.init_const(100, 0, 0, DEF_FRAC_DIGITS)

	x1 := fixed_bcd.init_const(50, 0, 0, DEF_FRAC_DIGITS)
	y1 := fixed_bcd.init_const(-50, 0, 0, DEF_FRAC_DIGITS)
	size2 := fixed_bcd.init_const(100, 0, 0, DEF_FRAC_DIGITS)

	square := _make_square_path_bcd(x0, y0, size)
	defer delete(square)

	square2 := _make_square_path_bcd(x1, y1, size2)
	defer delete(square2)

	poly := [][][2]fixed_bcd.BCD(DEF_FRAC_DIGITS){square}
	clip_poly := [][][2]fixed_bcd.BCD(DEF_FRAC_DIGITS){square2}
	res, res_open, err := BooleanOp_Fixed(.Intersection, DEF_FRAC_DIGITS, poly, clip_poly, nil)
	defer if res != nil {
		for r in res {
			delete(r)
		}
		delete(res)
	}
	defer if res_open != nil {
		for r in res_open {
			delete(r)
		}
		delete(res_open)
	}

	testing.expect_value(t, err, nil)
}


@(test)
test_custom :: proc(t: ^testing.T) {
	poly1 := [?][2]f64 {
		{340, 60}, // 0: 꼭대기
		{460, 380}, // 1: 오른쪽 아래 (→ 자가교차)
		{180, 200}, // 2: 왼쪽
		{500, 200}, // 3: 오른쪽
		{220, 380}, // 4: 왼쪽 아래 (→ 자가교차)
	}

	poly2: [len(poly1)][2]fixed_bcd.BCD(DEF_FRAC_DIGITS)

	for i in 0 ..< len(poly1) {
		poly2[i][0] = fixed_bcd.from_f64(DEF_FRAC_DIGITS, poly1[i][0])
		poly2[i][1] = fixed_bcd.from_f64(DEF_FRAC_DIGITS, poly1[i][1])
	}

	res, res_open, err := BooleanOp_Fixed(
		.Union,
		DEF_FRAC_DIGITS,
		[][][2]fixed_bcd.BCD(DEF_FRAC_DIGITS){poly2[:]},
		nil,
		nil,
		.NonZero,
	)
	defer {
		for r in res {
			delete(r)
		}
		delete(res)
	}

	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(res), 1)
	testing.expect_value(t, len(res[0]), 10)
}
