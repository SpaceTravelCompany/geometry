#+private
package linalg_ex

import "base:intrinsics"
import "core:math"
import "core:math/fixed"
import "engine:utils_private"
import "engine:utils_private/fixed_bcd"

NumAdd :: #force_inline proc "contextless" (
	a, b: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(
		T,
	) {return a + b} else when intrinsics.type_is_specialization_of(T, fixed.Fixed) {return fixed.add(a, b)} else {return fixed_bcd.add(a, b)}
}

NumSub :: #force_inline proc "contextless" (
	a, b: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(
		T,
	) {return a - b} else when intrinsics.type_is_specialization_of(T, fixed.Fixed) {return fixed.sub(a, b)} else {return fixed_bcd.sub(a, b)}
}

NumSign :: #force_inline proc "contextless" (
	a: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return -a} else {return T{i = -a.i}}
}

NumMul :: #force_inline proc "contextless" (
	a, b: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(
		T,
	) {return a * b} else when intrinsics.type_is_specialization_of(T, fixed.Fixed) {return fixed.mul(a, b)} else {return fixed_bcd.mul(a, b)}
}

NumDiv :: #force_inline proc "contextless" (
	a, b: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(
		T,
	) {return a / b} else when intrinsics.type_is_specialization_of(T, fixed.Fixed) {return fixed.div(a, b)} else {return fixed_bcd.div(a, b)}
}

NumLt :: #force_inline proc "contextless" (
	a, b: $T,
) -> bool where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a < b} else {return a.i < b.i}
}

NumGt :: #force_inline proc "contextless" (
	a, b: $T,
) -> bool where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a > b} else {return a.i > b.i}
}

NumLe :: #force_inline proc "contextless" (
	a, b: $T,
) -> bool where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a <= b} else {return a.i <= b.i}
}

NumGe :: #force_inline proc "contextless" (
	a, b: $T,
) -> bool where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a >= b} else {return a.i >= b.i}
}

NumEq :: #force_inline proc "contextless" (
	a, b: $T,
) -> bool where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a == b} else {return a.i == b.i}
}

NumMid :: #force_inline proc "contextless" (
	a, b: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {
		return (a + b) * T(0.5)
	} else when intrinsics.type_is_specialization_of(T, fixed.Fixed) {
		return T{i = (a.i + b.i) >> 1}
	} else {
		return fixed_bcd.div(fixed_bcd.add(a, b), NumConst(2, T))
	}
}

NumRatio :: #force_inline proc "contextless" (
	num, den: int,
	$T: typeid,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {
		return T(num) / T(den)
	} else when intrinsics.type_is_specialization_of(T, fixed.Fixed) {
		Frac :: intrinsics.type_polymorphic_record_parameter_value(T, 1)
		Backing :: intrinsics.type_polymorphic_record_parameter_value(T, 0)
		return T{i = (Backing(num) << Frac) / Backing(den)}
	} else {
		return fixed_bcd.div(NumConst(num, T), NumConst(den, T))
	}
}

NumConst :: #force_inline proc "contextless" (
	num: int,
	$T: typeid,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {
		return T(num)
	} else when intrinsics.type_is_specialization_of(T, fixed.Fixed) {
		Frac :: intrinsics.type_polymorphic_record_parameter_value(T, 1)
		Backing :: intrinsics.type_polymorphic_record_parameter_value(T, 0)
		return T{i = Backing(num) << Frac}
	} else {
		return fixed_bcd.init_const(
			num,
			0,
			0,
			intrinsics.type_polymorphic_record_parameter_value(T, 0),
		)
	}
}

NumAbs :: #force_inline proc "contextless" (
	v: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	zero := T(0) when intrinsics.type_is_float(T) else T{i = 0}
	if NumLt(v, zero) do return NumSub(zero, v)
	return v
}

NumSqrt :: #force_inline proc "contextless" (
	v: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {
		return math.sqrt(v)
	} else when intrinsics.type_is_specialization_of(T, fixed.Fixed) {
		n := u128(v.i) << u128(T.Fraction_Width)
		if n > u128(max(u64)) do return T{i = T.Backing(utils_private.sqrt_128(n))}
		return T{i = T.Backing(utils_private.sqrt_64(n))}
	} else {
		n := u128(v.i) * u128(fixed_bcd._SCALE_TABLE[T.FRAC_DIGITS])
		if n > u128(max(u64)) do return T{i = T.Backing(utils_private.sqrt_128(n))}
		return T{i = T.Backing(utils_private.sqrt_64(n))}
	}
}

Vec2Dot :: #force_inline proc "contextless" (
	a, b: [2]$T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	return NumAdd(NumMul(a[0], b[0]), NumMul(a[1], b[1]))
}

Vec2Sub :: #force_inline proc "contextless" (
	a, b: [2]$T,
) -> [2]T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	return {NumSub(a[0], b[0]), NumSub(a[1], b[1])}
}

NumMin :: #force_inline proc "contextless" (
	a, b: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	return NumLt(a, b) ? a : b
}

NumMax :: #force_inline proc "contextless" (
	a, b: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_specialization_of(T, fixed_bcd.BCD) ||
	intrinsics.type_is_float(T) {
	return NumGt(a, b) ? a : b
}

