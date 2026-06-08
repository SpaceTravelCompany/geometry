package triangulation

import "core:testing"
import "shared:utils_private"


@(private)
_makeSquare :: proc(
	x0, y0, size: f32,
	allocator := context.allocator,
) -> [][2]f32 {
	result := utils_private.makeNonZeroedSlice([][2]f32, 4, allocator)
	half := size * 0.5
	// CCW winding (y-up)
	result[0] = {x0 - half, y0 - half}
	result[1] = {x0 + half, y0 - half}
	result[2] = {x0 + half, y0 + half}
	result[3] = {x0 - half, y0 + half}
	return result
}


@(test)
testTriangulationSquare :: proc(t: ^testing.T) {
	square := _makeSquare(0, 0, 10)
	defer delete(square)

	poly := [][][2]f32{square}
	indices, err := TrianguatePolygons(poly)
	defer delete(indices)

	testing.expect_value(t, err, nil)
	// square → 2 triangles = 6 indices
	testing.expect_value(t, len(indices), 6)
}

@(test)
testTriangulationTwoSquares :: proc(t: ^testing.T) {
	sq1 := _makeSquare(0, 0, 10)
	defer delete(sq1)
	sq2 := _makeSquare(100, 0, 10) // 떨어진 위치
	defer delete(sq2)

	poly := [][][2]f32{sq1, sq2}
	indices, err := TrianguatePolygons(poly)
	defer delete(indices)

	testing.expect_value(t, err, nil)
	// 2 squares → 4 triangles = 12 indices
	testing.expect_value(t, len(indices), 12)
}

@(test)
testTriangulationTriangle :: proc(t: ^testing.T) {
	tri := utils_private.makeNonZeroedSlice([][2]f32, 3, context.temp_allocator)
	tri[0] = {0, 0}
	tri[1] = {10, 0}
	tri[2] = {5, 10}

	poly := [][][2]f32{tri}
	indices, err := TrianguatePolygons(poly)
	defer delete(indices)

	testing.expect_value(t, err, nil)
	// triangle → 1 triangle = 3 indices
	testing.expect_value(t, len(indices), 3)
	// 모든 인덱스가 유효 범위 내
	for idx in indices {
		testing.expect(t, idx < 3, "index out of range")
	}
}
