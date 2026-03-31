package triangulation

import "core:math/fixed"
import "core:testing"
import "engine:utils_private"


@(private)
_make_square_path :: proc(
	x0, y0, size: FixedDef,
	invent := false,
	allocator := context.allocator,
) -> [][2]FixedDef {
	result := utils_private.make_non_zeroed_slice([][2]FixedDef, 4, allocator)
	two: FixedDef = FixedDef {
		i = 2 << FixedDef.Fraction_Width,
	}
	half := fixed.div(size, two)
	if !invent {
		result[0] = {fixed.add(x0, half), fixed.add(y0, half)}
		result[1] = {fixed.sub(x0, half), fixed.add(y0, half)}
		result[2] = {fixed.sub(x0, half), fixed.sub(y0, half)}
		result[3] = {fixed.add(x0, half), fixed.sub(y0, half)}
	} else {
		result[0] = {fixed.sub(x0, half), fixed.sub(y0, half)}
		result[1] = {fixed.add(x0, half), fixed.sub(y0, half)}
		result[2] = {fixed.add(x0, half), fixed.add(y0, half)}
		result[3] = {fixed.sub(x0, half), fixed.add(y0, half)}
	}
	return result
}


@(test)
test_triangulation_2square :: proc(t: ^testing.T) {
	x0 := FixedDef {
		i = 2 << FixedDef.Fraction_Width,
	}
	y0 := FixedDef {
		i = 0,
	}
	size := FixedDef {
		i = 100 << FixedDef.Fraction_Width,
	}
	x1 := FixedDef {
		i = 25 << FixedDef.Fraction_Width,
	}
	y1 := FixedDef {
		i = -25 << FixedDef.Fraction_Width,
	}
	size2 := FixedDef {
		i = 50 << FixedDef.Fraction_Width,
	}

	square := _make_square_path(x0, y0, size)
	defer delete(square)

	square2 := _make_square_path(x1, y1, size2)
	defer delete(square2)

	poly := [][][2]FixedDef{square, square2}
	indices, err := TrianguatePolygons_Fixed(poly)
	defer delete(indices)

	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(indices), 12)
}

