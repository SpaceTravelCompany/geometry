#+feature using-stmt
package geometry

import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:math/linalg"
import "core:mem"

import "core:math/fixed"

__Trianguate_Error :: enum {
	FAILED,
}

Trianguate_Error :: union #shared_nil {
    __Trianguate_Error,
    runtime.Allocator_Error,
}

TrianguatePolygons_Fixed :: proc(poly:[][2]FixedDef, nPolys:[]u32, polyCCW:[]PolyOrientation = nil, allocator := context.allocator) ->
(indices:[]u32, err:Trianguate_Error) {
	indices_ := make([dynamic]u32, context.temp_allocator) or_return
	defer delete(indices_)
	
	//TODO non_curves trianglation
	
	indices = make_non_zeroed_slice([]u32, len(indices_), allocator) or_return
	mem.copy_non_overlapping(raw_data(indices), raw_data(indices_), len(indices_) * size_of(u32))
	return
}
