import math

#flag -I @VMODROOT\include\box2d
// #flag -IC:\Users\navez\Downloads\box2c\src

#include "box2d.h"

#flag -L @VMODROOT\include]bin

$if windows {
	#flag -l:libbox2d.a
}

// start math_functions

const b2_pi = 3.14159265359

// 2D vector
// This can be used to represent a point or free vector
@[typedef]
struct C.b2Vec2 {
pub mut:
	// coordinates
	x f32
	y f32
}

pub type B2Vec2 = C.b2Vec2

// 2D rotation
// This is similar to using a complex number for rotation
@[typedef]
struct C.b2Rot {
pub mut:
	// cosine and sine
	c f32
	s f32
} 

pub type B2Rot = C.b2Rot

// A 2D rigid transform
@[typedef]
struct C.b2Transform {
pub mut:
	p B2Vec2
	q B2Rot
}

pub type B2Transform = C.b2Transform

// A 2-by-2 Matrix
@[typedef]
struct C.b2Mat22 {
pub mut:
	// columns
	cx B2Vec2
	cy B2Vec2
}

pub type B2Mat22 = C.b2Mat22

// Axis-aligned bounding box
@[typedef]
struct C.b2AABB {
pub mut:
	lowerBound B2Vec2 
	upperBound B2Vec2
}

pub type B2AABB = C.b2AABB

pub const b2_vec2_zero = B2Vec2{}
pub const b2_rot_identity = B2Rot{1.0, 0.0}
pub const b2_transform_identity = B2Transform{B2Vec2{0.0, 0.0}, B2Rot{1.0, 0.0}}
pub const b2_mat22_zero = B2Mat22{}

// return the minimum of two floats
fn C.b2MinFloat(a f32, b f32) f32
@[inline]
pub fn b2_min_float(a f32, b f32) f32 {
	return C.b2MinFloat(a, b)
}

// @return the maximum of two floats
fn C.b2MaxFloat(a f32, b f32) f32
@[inline]
pub fn b2_max_float(a f32, b f32) f32 {
	return C.b2MaxFloat(a, b)
}

// return the absolute value of a float
fn C.b2AbsFloat(a f32) f32
@[inline]
pub fn b2_abs_float(a f32) f32 {
	return C.b2AbsFloat(a)
}

// return a clamped between a lower and upper bound
fn C.b2ClampFloat(a f32, lower f32, upper f32) f32
@[inline]
pub fn b2_clamp_float(a f32, lower f32, upper f32) f32 {
	return C.b2ClampFloat(a, lower, upper)
}

// return the minimum of two integers
fn C.b2MinInt(a int, b int) int
@[inline]
pub fn b2_min_int(a int, b int) int {
	return C.b2MinInt(a, b)
}

// return the maximum of two integers
fn C.b2MaxInt(a int, b int) int
@[inline]
pub fn b2_max_int(a int, b int) int {
	return C.b2MaxInt(a, b)
}

// return the absolute value of an integer
fn C.b2AbsInt(a int) int
@[inline]
pub fn b2_abs_int(a int) int {
	return C.b2AbsInt(a)
}

// return an integer clamped between a lower and upper bound
fn C.b2ClampInt(a int, lower int, upper int) int
@[inline]
pub fn b2_clamp_int(a int, lower int, upper int) int {
	return C.b2ClampInt(a, lower, upper)
}

// Vector dot product
fn C.b2Dot(a B2Vec2, b B2Vec2) f32
@[inline]
pub fn b2_dot(a B2Vec2, b B2Vec2) f32 {
	return C.b2Dot(a, b)
}

// Vector cross product. In 2D this yields a scalar.
fn C.b2Cross(a B2Vec2, b B2Vec2) f32
@[inline]
pub fn b2_cross(a B2Vec2, b B2Vec2) f32 {
	return C.b2Cross(a, b)
}

// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
@[inline]
pub fn b2_cross_vs(v B2Vec2, s f32) &B2Vec2 {
	return &B2Vec2{s * v.y, -s * v.x}
}

/// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
@[inline]
pub fn b2_cross_sv(s f32, v B2Vec2) &B2Vec2 {
	return &B2Vec2{-s * v.y, s * v.x}
}

// Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v)
@[inline]
pub fn b2_left_perp(v B2Vec2) &B2Vec2 {
	return &B2Vec2{-v.y, v.x}
}

// Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)
@[inline]
pub fn b2_right_perp(v B2Vec2) &B2Vec2 {
	return &B2Vec2{v.y, -v.x}
}

// Vector addition
@[inline]
pub fn b2_add(a B2Vec2, b B2Vec2) &B2Vec2 {
	return &B2Vec2{a.x + b.x, a.y + b.y}
}

// Vector subtraction
@[inline]
pub fn b2_sub(a B2Vec2, b B2Vec2) &B2Vec2 {
	return &B2Vec2{a.x - b.x, a.y - b.y}
}

// Vector negation
@[inline]
pub fn b2_neg(a B2Vec2) &B2Vec2 {
	return &B2Vec2{-a.x, -a.y}
}

// Vector linear interpolation
// https://fgiesen.wordpress.com/2012/08/
@[inline]
pub fn b2_lerp(a B2Vec2, b B2Vec2, t f32) &B2Vec2 {
	return &B2Vec2{(1.0 - t) * a.x + t * b.x, (1.0 - t) * a.y + t * b.y}
}

// Component-wise multiplication
@[inline]
pub fn b2_mul(a B2Vec2, b B2Vec2) &B2Vec2 {
	return &B2Vec2{a.x * b.x, a.y * b.y}
}

// Multiply a scalar and vector
@[inline]
pub fn b2_mul_sv(s f32, v B2Vec2) &B2Vec2 {
	return &B2Vec2{s * v.x, s * v.y}
}

// a + s * b
@[inline]
pub fn b2_mul_add(a B2Vec2, s f32, b B2Vec2) &B2Vec2 {
	return &B2Vec2{a.x + s * b.x, a.y + s * b.y}
}

// a - s * b
@[inline]
pub fn b2_mul_sub(a B2Vec2, s f32, b B2Vec2) &B2Vec2 {
	return &B2Vec2{a.x - s * b.x, a.y - s * b.y}
}

// Component-wise absolute vector
@[inline]
pub fn b2_abs(a B2Vec2) &B2Vec2 {
	mut b := B2Vec2{}
	b.x = b2_abs_float(a.x)
	b.y = b2_abs_float(a.y)
	return &b
}

// Component-wise minimum vector
@[inline]
pub fn b2_min(a B2Vec2, b B2Vec2) &B2Vec2 {
	mut c := B2Vec2{}
	c.x = b2_min_float(a.x, b.x)
	c.y = b2_min_float(a.y, b.y)
	return &c
}

// Component-wise maximum vector
@[inline]
pub fn b2_max(a B2Vec2, b B2Vec2) &B2Vec2 {
	mut c := B2Vec2{}
	c.x = b2_max_float(a.x, b.x)
	c.y = b2_max_float(a.y, b.y)
	return &c
}

// Component-wise clamp vector v into the range [a, b]
@[inline]
pub fn b2_clamp(v B2Vec2, a B2Vec2, b B2Vec2) &B2Vec2 {
	mut c := B2Vec2{}
	c.x = b2_clamp_float(v.x, a.x, b.x)
	c.y = b2_clamp_float(v.y, a.y, b.y)
	return &c
}

// Get the length of this vector (the norm)
fn C.b2Length(v B2Vec2) f32
@[inline]
pub fn b2_length(v B2Vec2) f32 {
	return C.b2Length(v)
}

// Get the length squared of this vector
fn C.b2LengthSquared(v B2Vec2) f32
@[inline]
pub fn b2_length_squared(v B2Vec2) f32 {
	return C.b2LengthSquared(v)
}

// Get the distance between two points
fn C.b2Distance(a B2Vec2, b B2Vec2) f32
@[inline]
pub fn b2_distance(a B2Vec2, b B2Vec2) f32 {
	return C.b2Distance(a, b)
}

// Get the distance squared between two points
fn C.b2DistanceSquared(a B2Vec2, b B2Vec2) f32
@[inline]
pub fn b2_distance_squared(a B2Vec2, b B2Vec2) f32 {
	return C.b2DistanceSquared(a, b)
}

// Set using an angle in radians
@[inline]
pub fn b2_make_rot(angle f32) &B2Rot {
	// todo determinism
	q := B2Rot{math.cosf(angle), math.sinf(angle)}
	return &q
}

// Normalize rotation
@[inline]
pub fn b2_normalize_rot(q B2Rot) &B2Rot {
	mut mag := math.sqrtf(q.s * q.s + q.c * q.c)
	mut inv_mag := if mag > 0.0 {f32(1.0)} else {f32(0.0)}
	qn := B2Rot{q.c * inv_mag, q.s * inv_mag}
	return &qn

}

// Is this rotation normalized?
fn C.b2IsNormalized(q B2Rot) bool
@[inline]
pub fn b2_is_normalized(q B2Rot) bool {
	return C.b2IsNormalized(q)
}

// Normalized linear interpolation
// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
@[inline]
pub fn b2_n_lerp(q1 B2Rot, q2 B2Rot, t f32) &B2Rot {
	omt := f32(1.0 - t)
	q := B2Rot{omt * q1.c + t * q2.c, 
	                  omt * q1.s + t * q2.s}
	return b2_normalize_rot(q)
}

// Integration rotation from angular velocity
//	q1 initial rotation
//	deltaAngle the angular displacement in radians
@[inline]
pub fn b2_integration_rotation(q1 B2Rot, delta_angle f32) &B2Rot {
	q2 := B2Rot{q1.c - delta_angle * q1.s, q1.s + delta_angle * q1.c}
	mag := f32(math.sqrtf(q2.s * q2.s + q2.c * q2.c))
	inv_mag := if mag > 0.0 {1.0 / mag} else {0.0}
	qn := B2Rot{q2.c * inv_mag, q2.s * inv_mag}
	return &qn
}

// Compute the angular velocity necessary to rotate between two rotations over a give time
//	q1 initial rotation
//	q2 final rotation
//	inv_h inverse time step
fn C.b2ComputeAngularVelocity(q1 B2Rot, q2 B2Rot, inv_h f32) f32
@[inline]
pub fn b2_compute_angular_velocity(q1 B2Rot, q2 B2Rot, inv_h f32) f32 {
	return C.b2ComputeAngularVelocity(q1, q2, inv_h)
}

// Get the angle in radians
fn C.b2Rot_GetAngle(q B2Rot) f32
@[inline]
pub fn b2_rot_get_angle(q B2Rot) f32 {
	return C.b2Rot_GetAngle(q)
}

// Get the x-axis
@[inline]
pub fn b2_rot_get_x_axis(q B2Rot) &B2Vec2 {
	v := B2Vec2 {q.c, q.s}
	return &v
}

// Get the y-axis
@[inline]
pub fn b2_rot_get_y_axis(q B2Rot) &B2Vec2 {
	v := B2Vec2{-q.s, q.c}
	return &v
}

// Multiply two rotations: q * r
@[inline]
pub fn b2_mul_rot(q B2Rot, r B2Rot) &B2Rot {
	mut qr := B2Rot{} 
	qr.s = q.s * r.c + q.c * r.s
	qr.c = q.c * r.c - q.s * r.s
	return &qr
}

// Transpose multiply two rotations: qT * r
@[inline]
pub fn b2_inv_mul_rot(q B2Rot, r B2Rot) &B2Rot {
	mut qr := B2Rot{} 
	qr.s = q.c * r.s - q.s * r.c
	qr.c = q.c * r.c + q.s * r.s
	return &qr
}

// relative angle between b and a (rot_b * inv(rot_a))// Transpose multiply two rotations: qT * r
fn C.b2RelativeAngle(b B2Rot, a B2Rot) f32
@[inline]
pub fn b2_relative_angle(b B2Rot, a B2Rot) f32 {
	return C.b2RelativeAngle(b, a)
}

// Convert an angle in the range [-2*pi, 2*pi] into the range [-pi, pi]
fn C.b2UnwindAngle(angle f32) f32
@[inline]
pub fn b2_unwind_angle(angle f32) f32 {
	return C.b2UnwindAngle(angle)
}

// Rotate a vector
@[inline]
pub fn b2_rotate_vector(q B2Rot, v B2Vec2) &B2Vec2 {
	return &B2Vec2{q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y}
}

// Inverse Rotate a vector
@[inline]
pub fn b2_inv_rotate_vector(q B2Rot, v B2Vec2) &B2Vec2 {
	return &B2Vec2{q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y}
}


// Transform a point (e.g. local space to world space)
@[inline]
pub fn b2_transform_point(t B2Transform, p B2Vec2) &B2Vec2 {
	x := f32(t.q.c * p.x - t.q.s * p.y) + t.p.x
	y := f32(t.q.s * p.x + t.q.c * p.y) + t.p.y

	return &B2Vec2{x, y}
}


// Inbere Transform a point (e.g. local space to world space)
@[inline]
pub fn b2_inv_transform_point(t B2Transform, p B2Vec2) &B2Vec2 {
	vx := f32(p.x - t.p.x)
	vy := f32(p.y - t.p.y)
	return &B2Vec2{t.q.c * vx + t.q.s * vy, -t.q.s * vx + t.q.c * vy}

}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
@[inline]
pub fn b2_mul_transforms(a B2Transform, b B2Transform) &B2Transform {
	mut c := B2Transform{}
	c.q = b2_mul_rot(a.q, b.q)
	c.p = b2_add(b2_rotate_vector(a.q, b.p), a.p)
	return &c
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
@[inline]
pub fn b2_inv_mul_transforms(a B2Transform, b B2Transform) &B2Transform {
	mut c := B2Transform{}  
	c.q = b2_inv_mul_rot(a.q, b.q)
	c.p = b2_inv_rotate_vector(a.q, b2_sub(b.p, a.p))
	return &c
}

/// Multiply a 2-by-2 matrix times a 2D vector
@[inline]
pub fn b2_mul_mv(a B2Mat22, v B2Vec2) &B2Vec2 {
	u := B2Vec2{a.cx.x * v.x + a.cy.x * v.y,
	                    a.cx.y * v.x + a.cy.y * v.y}
	return &u
}

/// Get the inverse of a 2-by-2 matrix
@[inline]
pub fn b2_get_inverse_22(z B2Mat22) &B2Mat22 {
	a := z.cx.x
	b := z.cy.x
	c := z.cx.y
	d := z.cy.y
	mut det :=  a * d - b * c
	if det != 0.0 {
		det = 1.0 / det
	}

	y := B2Mat22{B2Vec2{det * d, -det * c},
		                  B2Vec2{-det * b, det * a}}
	return &y
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
@[inline]
pub fn b2_solve_22(a B2Mat22, b B2Vec2) &B2Vec2 {
	a11 := f32(a.cx.x) 
	a12 := f32(a.cy.x) 
	a21 := f32(a.cx.y) 
	a22 := f32(a.cy.y)
	mut det := a11 * a22 - a12 * a21
	if det != 0.0 {
		det = 1.0 / det
	}
	x := B2Vec2{det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x)}
	return &x
}

/// Does a fully contain b
fn C.b2AABB_Contains(a B2AABB, b B2AABB) bool
@[inline]
pub fn b2_aabb_contains(a B2AABB, b B2AABB) bool {
	return C.b2AABB_Contains(a, b)
}

/// Get the center of the AABB.
@[inline]
pub fn b2_aabb_center(a B2AABB) &B2Vec2 {
	return &B2Vec2{0.5 * (a.lowerBound.x + a.upperBound.x), 0.5 * (a.lowerBound.y + a.upperBound.y)}
}

@[inline]
pub fn b2_aabb_extents(a B2AABB) &B2Vec2 {
	return &B2Vec2{0.5 * (a.upperBound.x - a.lowerBound.x), 0.5 * (a.upperBound.y - a.lowerBound.y)}

}

@[inline]
pub fn b2_aabb_union(a B2AABB, b B2AABB) &B2AABB {
	mut c := B2AABB{} 
	c.lowerBound.x = b2_min_float(a.lowerBound.x, b.lowerBound.x)
	c.lowerBound.y = b2_min_float(a.lowerBound.y, b.lowerBound.y)
	c.upperBound.x = b2_max_float(a.upperBound.x, b.upperBound.x)
	c.upperBound.y = b2_max_float(a.upperBound.y, b.upperBound.y)
	return &c
}

/// Is this a valid number? Not NaN or infinity.
fn C.b2IsValid(a f32) bool
@[inline]
pub fn b2_is_valid(a f32) bool {
	return C.b2IsValid(a)
}

/// Is this a valid vector? Not NaN or infinity.
fn C.b2Vec2_IsValid(v B2Vec2) bool
@[inline]
pub fn b2_vec2_is_valid(v B2Vec2) bool {
	return C.b2Vec2_IsValid(v)
}

/// Is this a valid rotation? Not NaN or infinity. Is normalized.
fn C.b2Rot_IsValid(q B2Rot) bool
@[inline]
pub fn b2_rot_is_valid(q B2Rot) bool {
	return C.b2Rot_IsValid(q)
}

/// Is this a valid bounding box? Not Nan or infinity. Upper bound greater than or equal to lower bound.
fn C.b2AABB_IsValid(aabb B2AABB) bool
@[inline]
pub fn b2_aabb_is_valid(aabb B2AABB) bool {
	return C.b2AABB_IsValid(aabb)
}

/// Is this a valid vector? Not NaN or infinity.
fn C.b2Vec2_IsValid(v B2Vec2) bool
@[inline]
pub fn b2_is_vec2_valid(v B2Vec2) bool {
	return C.b2Vec2_IsValid(v)
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
fn C.b2Normalize(v B2Vec2) B2Vec2
@[inline]
pub fn b2_normalize(v B2Vec2) B2Vec2 {
	return C.b2Normalize(v)
}

/// Convert a vector into a unit vector if possible, otherwise asserts.
fn C.b2NormalizeChecked(v B2Vec2) B2Vec2
@[inline]
pub fn b2_normalize_checked(v B2Vec2) B2Vec2 {
	return C.b2NormalizeChecked(v)
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also
///	outputs the length.
fn C.b2GetLengthAndNormalize(length &f32, v B2Vec2) B2Vec2
@[inline]
pub fn b2_get_length_and_normalize(length &f32, v B2Vec2) B2Vec2 {
	return C.b2GetLengthAndNormalize(length, v)
}

/// Box2D bases all length units on meters, but you may need different units for your game.
/// You can set this value to use different units. This should be done at application startup
///	and only modified once. Default value is 1.
///	@warning This must be modified before any calls to Box2D
fn C.b2SetLengthUnitsPerMeter(lengthUnits f32)
@[inline]
pub fn b2_set_length_units_per_meter(lengthUnits f32) {
	C.b2SetLengthUnitsPerMeter(lengthUnits)
}

// overloads below

//overloads '+' operator to add 2 vecs
fn (a B2Vec2) + (b B2Vec2) &B2Vec2 {
	return &B2Vec2{a.x + b.x, a.y + b.y}
}

//overloads '-' operator to subract 2 vecs
fn (a B2Vec2) - (b B2Vec2) &B2Vec2 {
	return &B2Vec2{a.x - b.x, a.y - b.y}
}

//overloads '*' operator to multiply 2 vecs
fn (a B2Vec2) * (b B2Vec2) &B2Vec2 {
	return &B2Vec2{a.x * b.x, a.y * b.y}
}

//overloads '/' operator to divide 2 vecs
fn (a B2Vec2) / (b B2Vec2) &B2Vec2 {
	return &B2Vec2{a.x / b.x, a.y / b.y}
}

//overloads '==' operator to compare two vecs
fn (a B2Vec2) == (b B2Vec2) bool {
	return a.x == b.x && a.y == b.y
}

// end math_functions

// start base

/// Prototype for user allocation function
pub type B2AllocFcn = fn(u32, int) voidptr
/// Prototype for user free function
pub type B2FreeFcn = fn(voidptr)
/// Prototype for the user assert callback. Return 0 to skip the debugger break.
pub type B2AssertFcn = fn(&char, &char, int) int

/// This allows the user to override the allocation functions. These should be
///	set during application startup.
fn C.b2SetAllocator(allocFcn B2AllocFcn, freeFcn B2FreeFcn)
@[inline]
pub fn b2_set_allocator(allocFcn B2AllocFcn, freeFcn B2FreeFcn) {
	C.b2SetAllocator(allocFcn, freeFcn)
}

/// @return the total bytes allocated by Box2D
fn C.b2GetByteCount() int
@[inline]
pub fn b2_get_byte_count() int {
	return C.b2GetByteCount()
}

/// Override the default assert callback
///	assertFcn a non-null assert callback
fn C.b2SetAssertFcn(assertFcn &B2AssertFcn)
@[inline]
pub fn b2_set_assert_fcn(assertFcn &B2AssertFcn) {
	C.b2SetAssertFcn(assertFcn)
}

/// Get the current version of Box2D
fn C.b2GetVersion()
@[inline]
pub fn b2_get_version() {
	C.b2GetVersion()
}

fn C.b2CreateTimer() B2Timer
@[inline]
pub fn b2_create_timer() B2Timer {
	return C.b2CreateTimer()
}

fn C.b2GetTicks(timer &B2Timer) i64
@[inline]
pub fn b2_get_ticks(timer &B2Timer) i64 {
	return C.b2GetTicks(timer)
}

fn C.b2GetMilliseconds(timer &B2Timer) f32
@[inline]
pub fn b2_get_milli_seconds(timer &B2Timer) f32 {
	return C.b2GetMilliseconds(timer)
}

fn C.b2GetMillisecondsAndReset(timer &B2Timer) f32
@[inline]
pub fn b2_get_milli_seconds_and_reset(timer &B2Timer) f32 {
	return C.b2GetMillisecondsAndReset(timer)
}

fn C.b2SleepMilliseconds(milliseconds int)
@[inline]
pub fn b2_sleep_milliseconds(milliseconds int) {
	C.b2SleepMilliseconds(milliseconds)
}

/// Version numbering scheme.
/// See https://semver.org/
@[typedef]
struct C.b2Version {
pub mut:
	// coordinates
	major int
	minor int
	revision int
}

pub type B2Version = C.b2Version

// to do. figure out how to implement linux and mac versions

// for windows
// Timer for profiling. This has platform specific code and may not work on every platform.
@[typedef]
struct C.b2Timer {
pub mut:
	// coordinate
	start i64
}

pub type B2Timer = C.b2Timer

// end base

// start id 

/// World id references a world instance. This should be treated as an opaque handle.
@[typedef]
struct C.b2WorldId {
pub mut:
	index1 u16
	revision u16
}

pub type B2WorldId = C.b2WorldId

/// Body id references a body instance. This should be treated as an opaque handle.
@[typedef]
struct C.b2BodyId {
pub mut:
	index1 int
	world0 u16
	revision u16
}

pub type B2BodyId = C.b2BodyId

/// Shape id references a shape instance. This should be treated as an opaque handle.
@[typedef]
struct C.b2ShapeId {
pub mut:
	index1 int
	world0 u16
	revision u16
}

pub type B2ShapeId = C.b2ShapeId

/// Joint id references a joint instance. This should be treated as an opaque handle.
@[typedef]
struct C.b2JointId {
pub mut:
	index1 int
	world0 u16
	revision u16
}

pub type B2JointId = C.b2JointId

/// Chain id references a chain instances. This should be treated as an opaque handle.
@[typedef]
struct C.b2ChainId {
pub mut:
	index1 int
	world0 u16
	revision u16
}

pub type B2ChainId = C.b2ChainId

pub const b2_null_world_id =  B2WorldId{0, 0}
pub const b2_null_body_id = B2BodyId{0, 0, 0}
pub const b2_null_shape_id = B2ShapeId{0, 0, 0}
pub const b2_null_joint_id = B2JointId{0, 0, 0}
pub const b2_null_chain_id = B2ChainId{0, 0, 0}

@[inline]
fn b2_is_null[T](id T) bool {
	return id.index1 == 0
}

@[inline]
fn b2_is_non_null[T](id T) bool {
	return id.index1 != 0
}

@[inline]
fn b2_id_equals[T](id1 T, id2 T) bool {
	return id1.index1 == id2.index1 && id1.world0 == id2.world0 && id1.revision == id2.revision
}

// end id

// start collision

const b2_max_polygon_vertices = 8

/// Low level ray-cast input data
@[typedef]
struct C.b2RayCastInput {
pub mut:
	/// Start point of the ray cast
	origin B2Vec2

	/// Translation of the ray cast
	translation B2Vec2

	/// The maximum fraction of the translation to consider, typically 1
	maxFraction f32
}

pub type B2RayCastInput = C.b2RayCastInput

/// Low level shape cast input in generic form. This allows casting an arbitrary point
///	cloud wrap with a radius. For example, a circle is a single point with a non-zero radius.
///	A capsule is two points with a non-zero radius. A box is four points with a zero radius.
@[typedef]
struct C.b2ShapeCastInput {
pub mut:
	/// A point cloud to cast
	points [b2_max_polygon_vertices]B2Vec2

	/// The number of points
	count int

	/// The radius around the point cloud
	radius f32

	/// The translation of the shape cast
	translation B2Vec2 

	/// The maximum fraction of the translation to consider, typically 1
	maxFraction f32
}

pub type B2ShapeCastInput = C.b2ShapeCastInput


/// Low level ray-cast or shape-cast output data
@[typedef]
struct C.b2CastOutput {
pub mut:
	/// The surface normal at the hit point
	normal B2Vec2

	/// The surface hit point
	point B2Vec2

	/// The fraction of the input translation at collision
	fraction f32

	/// The number of iterations used
	iterations int

	/// Did the cast hit?
	hit int
}

pub type B2CastOutput = C.b2CastOutput

/// This holds the mass data computed for a shape.
@[typedef]
struct C.b2MassData {
pub mut:
	/// The mass of the shape, usually in kilograms.
	mass f32

	/// The position of the shape's centroid relative to the shape's origin.
	center B2Vec2 

	/// The rotational inertia of the shape about the local origin.
	rotationalInertia f32
}

pub type B2MassData = C.b2MassData

/// A solid circle
@[typedef]
struct C.b2Circle {
pub mut:
	/// The local center
	center B2Vec2

	/// The radius
	radius f32
}

pub type B2Circle = C.b2Circle

/// A solid capsule can be viewed as two semicircles connected
///	by a rectangle.
@[typedef]
struct C.b2Capsule {
	/// Local center of the first semicircle
	center1 B2Vec2
	
	/// Local center of the second semicircle
	center2 B2Vec2

	/// The radius of the semicircles
	radius f32
}

pub type B2Capsule = C.b2Capsule

/// A solid convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
///	@warning DO NOT fill this out manually, instead use a helper function like
///	b2MakePolygon or b2MakeBox.
@[typedef] 
struct C.b2Polygon {
	/// The polygon vertices
	vertices [b2_max_polygon_vertices]B2Vec2 

	/// The outward normal vectors of the polygon sides
	normals [b2_max_polygon_vertices]B2Vec2 

	/// The centroid of the polygon
	centroid B2Vec2

	/// The external radius for rounded polygons
	radius f32

	/// The number of polygon vertices
	count int
}

pub type B2Polygon = C.b2Polygon

/// A line segment with two-sided collision.
@[typedef] 
struct C.b2Segment {
	/// The first point
	point1 B2Vec2

	/// The second point
	point2 B2Vec2
} 

pub type B2Segment = C.b2Segment

/// A smooth line segment with one-sided collision. Only collides on the right side.
/// Several of these are generated for a chain shape.
/// ghost1 -> point1 -> point2 -> ghost2
@[typedef] 
struct C.b2SmoothSegment {
	/// The tail ghost vertex
	ghost1 B2Vec2

	/// The line segment
	segment B2Segment 

	/// The head ghost vertex
	ghost2 B2Vec2 

	/// The owning chain shape index (internal usage only)
	chainId int
}

pub type B2SmoothSegment = C.b2SmoothSegment

/// A convex hull. Used to create convex polygons.
@[typedef] 
struct C.b2Hull {
	/// The final points of the hull
	points [b2_max_polygon_vertices]B2Vec2

	/// The number of points
	count int
} 

pub type B2Hull = C.b2Hull

/// Result of computing the distance between two line segments
@[typedef] 
struct C.b2SegmentDistanceResult {
	/// The closest point on the first segment
	closest1 B2Vec2 

	/// The closest point on the second segment
	closest2 B2Vec2

	/// The barycentric coordinate on the first segment
	fraction1 f32

	/// The barycentric coordinate on the second segment
	fraction2 f32

	/// The squared distance between the closest points
	distanceSquared f32
}

pub type B2SegmentDistanceResult = C.b2SegmentDistanceResult

/// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
@[typedef] 
struct C.b2DistanceProxy {
	/// The point cloud
	points [b2_max_polygon_vertices]B2Vec2

	/// The number of points
	count int

	/// The external radius of the point cloud
	radius f32
}

pub type B2DistanceProxy = C.b2DistanceProxy

// Used to warm start b2Distance. Set count to zero on first call or
///	use zero initialization.
@[typedef] 
struct C.b2DistanceCache {
	
	/// The number of stored simplex points
	count u16

	/// The cached simplex indices on shape A
	indexA [3]u8

	/// The cached simplex indices on shape B
	indexB [3]u8
}

pub type B2DistanceCache = C.b2DistanceCache

pub const b2_empty_distance_cache = B2DistanceCache{}

// Input for b2ShapeDistance
@[typedef] 
struct C.b2DistanceInput {
	/// The proxy for shape A
	proxyA B2DistanceProxy 

	/// The proxy for shape B
	proxyB B2DistanceProxy

	/// The world transform for shape A
	transformA B2Transform 

	/// The world transform for shape B
	transformB B2Transform

	/// Should the proxy radius be considered?
	use_radii bool
}

pub type B2DistanceInput = C.b2DistanceInput

/// Output for b2ShapeDistance
@[typedef] 
struct C.b2DistanceOutput {
	pointA B2Vec2  ///< Closest point on shapeA
	pointB B2Vec2  ///< Closest point on shapeB
	distance f32	///< The final distance, zero if overlapped
	iterations int ///< Number of GJK iterations used
	simplexCount int ///< The number of simplexes stored in the simplex array
}

pub type B2DistanceOutput = C.b2DistanceOutput

/// Simplex vertex for debugging the GJK algorithm
@[typedef] 
struct C.b2SimplexVertex {
	wA B2Vec2 	///< support point in proxyA
	wB B2Vec2 	///< support point in proxyB
	w B2Vec2	///< wB - wA
	a f32	///< barycentric coordinate for closest point
	indexA int ///< wA index
	indexB int ///< wB index
}

pub type B2SimplexVertex = C.b2SimplexVertex

/// Simplex from the GJK algorithm
@[typedef] 
struct C.b2Simplex {
	/// vertices
	v1 B2SimplexVertex
	v2 B2SimplexVertex
	v3 B2SimplexVertex

	count int ///< number of valid vertices
}

pub type B2Simplex = C.b2Simplex

/// Input parameters for b2ShapeCast
@[typedef] 
struct C.b2ShapeCastPairInput {
	proxyA B2DistanceProxy  ///< The proxy for shape A
	proxyB B2DistanceProxy ///< The proxy for shape B
	transformA B2Transform  ///< The world transform for shape A
	transformB B2Transform ///< The world transform for shape B
	translationB B2Vec2  ///< The translation of shape B
	maxFraction f32 ///< The fraction of the translation to consider, typically 1
}

pub type B2ShapeCastPairInput = C.b2ShapeCastPairInput

/// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
/// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
/// position.
@[typedef] 
struct C.b2Sweep {
	localCenter B2Vec2 ///< Local center of mass position
	c1 B2Vec2 ///< Starting center of mass world position
	c2 B2Vec2 ///< Ending center of mass world position
	q1 B2Rot  ///< Starting world rotation
	q2 B2Rot ///< Ending world rotation
}

pub type B2Sweep = C.b2Sweep

/// Input parameters for b2TimeOfImpact
@[typedef] 
struct C.b2TOIInput {
	proxyA B2DistanceProxy  ///< The proxy for shape A
	proxyB B2DistanceProxy ///< The proxy for shape B
	sweepA B2Sweep  ///< The movement of shape A
	sweepB B2Sweep ///< The movement of shape B
	tMax f32 ///< Defines the sweep interval [0, tMax]
}

pub type B2TOIInput = C.b2TOIInput

/// Describes the TOI output
pub enum B2TOIState {
	b2_to_i_state_unknown = 0
	b2_to_i_state_failed = 1
	b2_to_i_state_overlapped = 2
	b2_to_i_state_hit = 3
	b2_to_i_state_separated = 4
}

/// Output parameters for b2TimeOfImpact.
@[typedef] 
struct C.b2TOIOutput {
	state B2TOIState  ///< The type of result
	t f32 ///< The time of the collision
}

pub type B2TOIOutput = C.b2TOIOutput

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
@[typedef] 
struct C.b2ManifoldPoint {
	/// Location of the contact point in world space. Subject to precision loss at large coordinates.
	///	@note Should only be used for debugging.
	point B2Vec2 

	/// Location of the contact point relative to bodyA's origin in world space
	///	@note When used internally to the Box2D solver, these are relative to the center of mass.
	anchorA B2Vec2

	/// Location of the contact point relative to bodyB's origin in world space
	anchorB B2Vec2

	/// The separation of the contact point, negative if penetrating
	separation f32

	/// The impulse along the manifold normal vector.
	normalImpulse f32

	/// The friction impulse
	tangentImpulse f32

	/// The maximum normal impulse applied during sub-stepping
	///	todo not sure this is needed
	max_normalImpulse f32

	/// Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
	/// zero then there was no hit. Negative means shapes are approaching.
	normalVelocity f32

	/// Uniquely identifies a contact point between two shapes
	id u16

	/// Did this contact point exist the previous step?
	persisted bool
}

pub type B2ManifoldPoint = C.b2ManifoldPoint

/// A contact manifold describes the contact points between colliding shapes
@[typedef] 
struct C.b2Manifold {
	/// The manifold points, up to two are possible in 2D
	points [2]B2ManifoldPoint

	/// The unit normal vector in world space, points from shape A to bodyB
	normal B2Vec2 

	/// The number of contacts points, will be 0, 1, or 2
	pointCount int
}

pub type B2Manifold = C.b2Manifold

pub const (
	/// The default category bit for a tree proxy. Used for collision filtering.
	b2_default_category_bits = u64(0x00000001)

	/// Convenience mask bits to use when you don't need collision filtering and just want
	///	all results.
	b2_default_mask_bits = u64(0xFFFFFFFF)
)

/// A node in the dynamic tree. This is private data placed here for performance reasons.
/// 16 + 16 + 8 + pad(8)
@[typedef] 
struct C.b2TreeNode {
	/// The node bounding box
	aabb B2AABB  // 16

	/// Category bits for collision filtering
	categoryBits u32 // 4


	/// The node parent index
	parent int

	/// The node freelist next index
	next int

	/// Child 1 index
	child1 int  // 4

	/// Child 2 index
	child2 int // 4

	/// User data 
	// todo could be union with child index
	userData int // 4

	/// Leaf = 0, free node = -1
	height i16 // 2

	/// Has the AABB been enlarged?
	enlarged bool // 1

	/// Padding for clarity
	pad [9]char
}

pub type B2TreeNode = C.b2TreeNode

/// The dynamic tree structure. This should be considered private data.
/// It is placed here for performance reasons.
@[typedef] 
struct C.b2DynamicTree {
	/// The tree nodes
	nodes &B2TreeNode

	/// The root index
	root int

	/// The number of nodes
	nodeCount int

	/// The allocated node space
	nodeCapacity int

	/// Node free list
	freeList int

	/// Number of proxies created
	proxyCount int

	/// Leaf indices for rebuild
	leafIndices &int

	/// Leaf bounding boxes for rebuild
	leafBoxes &B2AABB 

	/// Leaf bounding box centers for rebuild
	leafCenters &B2Vec2 

	/// Bins for sorting during rebuild
	binIndices &int

	/// Allocated space for rebuilding
	rebuildCapacity int
}

pub type B2DynamicTree = C.b2DynamicTree


pub type B2TreeQueryCallbackFcn = fn(int, int voidptr) bool
pub type B2TreeRayCastCallbackFcn = fn(&B2RayCastInput, int, int voidptr) f32
pub type B2TreeShapeCastCallbackFcn = fn(&B2ShapeCastInput, int, int, voidptr) f32


/// Validate ray cast input data (NaN, etc)
fn C.b2IsValidRay(input &B2RayCastInput) bool
@[inline]
pub fn b2_is_valid_ray(input &B2RayCastInput) bool {
	return C.b2IsValidRay(input)
}

/// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
fn C.b2MakePolygon(hull &B2Hull, radius f32) B2Polygon
@[inline]
pub fn b2_make_polygon(hull &B2Hull, radius f32) B2Polygon {
	return C.b2MakePolygon(hull, radius)
}

/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
fn C.b2MakeOffsetPolygon(hull &B2Hull, radius f32, transform B2Transform) B2Polygon
@[inline]
pub fn b2_make_offset_polygon(hull &B2Hull, radius f32, transform B2Transform) B2Polygon {
	return C.b2MakePolygon(hull, radius)
}

/// Make a square polygon, bypassing the need for a convex hull.
fn C.b2MakeSquare(h f32) B2Polygon
@[inline]
pub fn b2_make_square(h f32) B2Polygon {
	return C.b2MakeSquare(h)
}

/// Make a box (rectangle) polygon, bypassing the need for a convex hull.
fn C.b2MakeBox(hx f32, hy f32) B2Polygon
@[inline]
pub fn b2_make_box(hx f32, hy f32) B2Polygon {
	return C.b2MakeBox(hx, hy)
}

/// Make a rounded box, bypassing the need for a convex hull.
fn C.b2MakeRoundedBox(hx f32, hy f32, radius f32) B2Polygon
@[inline]
pub fn b2_make_rounded_box(hx f32, hy f32, radius f32) B2Polygon {
	return C.b2MakeRoundedBox(hx, hy, radius)
}

/// Make an offset box, bypassing the need for a convex hull.
fn C.b2MakeOffsetBox(hx f32, hy f32, center B2Vec2, angle f32) B2Polygon
@[inline]
pub fn b2_make_offset_box(hx f32, hy f32, center B2Vec2, angle f32) B2Polygon {
	return C.b2MakeOffsetBox(hx, hy, center, angle)
}

/// Transform a polygon. This is useful for transferring a shape from one body to another.
fn C.b2TransformPolygon(transform B2Transform, polygon &B2Polygon) B2Polygon
@[inline]
pub fn b2_transform_polygon(transform B2Transform, polygon &B2Polygon) B2Polygon {
	return C.b2TransformPolygon(transform, polygon)
}

/// Compute mass properties of a circle
fn C.b2ComputeCircleMass(shape &B2Circle, density f32) B2MassData
@[inline]
pub fn b2_compute_circle_mass(shape &B2Circle, density f32) B2MassData {
	return C.b2ComputeCircleMass(shape, density)
}

/// Compute mass properties of a capsule
fn C.b2ComputeCapsuleMass(shape &B2Capsule, density f32) B2MassData
@[inline]
pub fn b2_compute_capsule_mass(shape &B2Capsule, density f32) B2MassData {
	return C.b2ComputeCapsuleMass(shape, density)
}

/// Compute mass properties of a polygon
fn C.b2ComputePolygonMass(shape &B2Polygon, density f32) B2MassData
@[inline]
pub fn b2_compute_polgon_mass(shape &B2Polygon, density f32) B2MassData {
	return C.b2ComputePolygonMass(shape, density)
}

/// Compute the bounding box of a transformed circle
fn C.b2ComputeCircleAABB(shape &B2Circle, transform B2Transform) B2AABB
@[inline]
pub fn b2_compute_circle_aabb(shape &B2Circle, transform B2Transform) B2AABB {
	return C.b2ComputeCircleAABB(shape, transform)
}

/// Compute the bounding box of a transformed capsule
fn C.b2ComputeCapsuleAABB(shape &B2Capsule, transform B2Transform) B2AABB
@[inline]
pub fn b2_compute_capsule_aabb(shape &B2Capsule, transform B2Transform) B2AABB {
	return C.b2ComputeCapsuleAABB(shape, transform)
}

/// Compute the bounding box of a transformed polygon
fn C.b2ComputePolygonAABB(shape &B2Polygon, transform B2Transform) B2AABB
@[inline]
pub fn b2_compute_polygon_aabb(shape &B2Polygon, transform B2Transform) B2AABB {
	return C.b2ComputePolygonAABB(shape, transform)
}

/// Compute the bounding box of a transformed line segment
fn C.b2ComputeSegmentAABB(shape &B2Segment, transform B2Transform) B2AABB
@[inline]
pub fn b2_compute_segment_aabb(shape &B2Segment, transform B2Transform) B2AABB {
	return C.b2ComputeSegmentAABB(shape, transform)
}

/// Test a point for overlap with a circle in local space
fn C.b2PointInCircle(point B2Vec2, shape &B2Circle) bool
@[inline]
pub fn b2_point_in_circle(point B2Vec2, shape &B2Circle) bool {
	return C.b2PointInCircle(point, shape)
}

/// Test a point for overlap with a convex polygon in local space
fn C.b2PointInPolygon(point B2Vec2, shape &B2Polygon) bool
@[inline]
pub fn b2_point_in_polygon(point B2Vec2, shape &B2Polygon) bool {
	return C.b2PointInPolygon(point, shape)
}

/// Ray cast versus circle in shape local space. Initial overlap is treated as a miss.
fn C.b2RayCastCircle(input &B2RayCastInput, shape &B2Circle) B2CastOutput
@[inline]
pub fn b2_ray_cast_circle(input &B2RayCastInput, shape &B2Circle) B2CastOutput {
	return C.b2RayCastCircle(input, shape)
}

/// Ray cast versus capsule in shape local space. Initial overlap is treated as a miss.
fn C.b2RayCastCapsule(input &B2RayCastInput, shape &B2Capsule) B2CastOutput
@[inline]
pub fn b2_ray_cast_capsule(input &B2RayCastInput, shape &B2Capsule) B2CastOutput {
	return C.b2RayCastCapsule(input, shape)
}

/// Ray cast versus segment in shape local space. Optionally treat the segment as one-sided with hits from
/// the left side being treated as a miss.
fn C.b2RayCastSegment(input &B2RayCastInput, shape &B2Segment, oneSided bool) B2CastOutput
@[inline]
pub fn b2_ray_cast_segment(input &B2RayCastInput, shape &B2Segment, oneSided bool) B2CastOutput {
	return C.b2RayCastSegment(input, shape, oneSided)
}

/// Ray cast versus polygon in shape local space. Initial overlap is treated as a miss.
fn C.b2RayCastPolygon(input &B2RayCastInput, shape &B2Polygon) B2CastOutput
@[inline]
pub fn b2_ray_cast_polygon(input &B2RayCastInput, shape &B2Polygon) B2CastOutput {
	return C.b2RayCastPolygon(input, shape)
}


/// Shape cast versus a circle. Initial overlap is treated as a miss.
fn C.b2ShapeCastCircle(input &B2ShapeCastInput, shape &B2Circle) B2CastOutput
@[inline]
pub fn b2_shape_cast_circle(input &B2ShapeCastInput, shape &B2Circle) B2CastOutput {
	return C.b2ShapeCastCircle(input, shape)
}

/// Shape cast versus a capsule. Initial overlap is treated as a miss.
fn C.b2ShapeCastCapsule(input &B2ShapeCastInput, shape &B2Capsule) B2CastOutput
@[inline]
pub fn b2_shape_cast_capsule(input &B2ShapeCastInput, shape &B2Capsule) B2CastOutput {
	return C.b2ShapeCastCapsule(input, shape)
}

/// Shape cast versus a line segment. Initial overlap is treated as a miss.
fn C.b2ShapeCastSegment(input &B2ShapeCastInput, shape &B2Segment) B2CastOutput
@[inline]
pub fn b2_shape_cast_segment(input &B2ShapeCastInput, shape &B2Segment) B2CastOutput {
	return C.b2ShapeCastSegment(input, shape)
}

/// Shape cast versus a convex polygon. Initial overlap is treated as a miss.
fn C.b2ShapeCastPolygon(input &B2ShapeCastInput, shape &B2Polygon) B2CastOutput
@[inline]
pub fn b2_shape_cast_polygon(input &B2ShapeCastInput, shape &B2Polygon) B2CastOutput {
	return C.b2ShapeCastPolygon(input, shape)
}

/// Test a point for overlap with a circle in local space
fn C.b2ComputeHull(points &B2Vec2, count int) B2Hull
@[inline]
pub fn b2_compute_hull(points &B2Vec2, count int) B2Hull {
	return C.b2ComputeHull(points, count)
}

/// This determines if a hull is valid. Checks for:
/// - convexity
/// - collinear points
/// This is expensive and should not be called at runtime.
fn C.b2ValidateHull(hull &B2Hull) bool
@[inline]
pub fn b2_validate_hull(hull &B2Hull) bool {
	return C.b2ValidateHull(hull)
}

/// Compute the distance between two line segments, clamping at the end points if needed.
fn C.b2SegmentDistance(p1 B2Vec2, q1 B2Vec2, p2 B2Vec2, q2 B2Vec2) B2SegmentDistanceResult 
@[inline]
pub fn b2_segment_distance(p1 B2Vec2, q1 B2Vec2, p2 B2Vec2, q2 B2Vec2) B2SegmentDistanceResult {
	return C.b2SegmentDistance(p1, q1, p2, q2)
}

/// Compute the closest points between two shapes represented as point clouds.
/// b2DistanceCache cache is input/output. On the first call set b2DistanceCache.count to zero.
///	The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. You may pass in NULL and 0 for these.
fn C.b2ShapeDistance(cache &B2DistanceCache, input &B2DistanceInput, simplexes &B2Simplex, simplexCapacity int)  B2DistanceOutput 
@[inline]
pub fn b2_shape_distance(cache &B2DistanceCache, input &B2DistanceInput, simplexes &B2Simplex, simplexCapacity int)  B2DistanceOutput {
	return C.b2ShapeDistance(cache, input, simplexes, simplexCapacity)
}

/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
fn C.b2ShapeCast(input &B2ShapeCastPairInput) B2CastOutput 
@[inline]
pub fn b2_shape_cast(input &B2ShapeCastPairInput) B2CastOutput  {
	return C.b2ShapeCast(input)
}

/// Make a proxy for use in GJK and related functions.
fn C.b2MakeProxy(vertices &B2Vec2, count int, radius f32) B2DistanceProxy
@[inline]
pub fn b2_make_proxy(vertices &B2Vec2, count int, radius f32) B2DistanceProxy  {
	return C.b2MakeProxy(vertices, count, radius)
}

/// Evaluate the transform sweep at a specific time.
fn C.b2GetSweepTransform(sweep &B2Sweep, time f32) B2Transform 
@[inline]
pub fn b2_get_sweep_transform(sweep &B2Sweep, time f32) B2Transform  {
	return C.b2GetSweepTransform(sweep, time)
}

/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collisions. If you change the time interval, you should call this function
/// again.
fn C.b2TimeOfImpact(input &B2TOIInput) B2TOIOutput  
@[inline]
pub fn b2_time_of_impact(input &B2TOIInput) B2TOIOutput  {
	return C.b2TimeOfImpact(input)
}

/// Compute the contact manifold between two circles
fn C.b2CollideCircles(circleA &B2Circle, xfA B2Transform, circleB &B2Circle, xfB B2Transform) B2Manifold 
@[inline]
pub fn b2_collide_circles(circleA &B2Circle, xfA B2Transform, circleB &B2Circle, xfB B2Transform) B2Manifold  {
	return C.b2CollideCircles(circleA, xfA, circleB, xfB)
}

/// Compute the contact manifold between a capsule and circle
fn C.b2CollideCapsuleAndCircle(capsuleA &B2Capsule, xfA B2Transform, circleB &B2Circle, xfB B2Transform) B2Manifold 
@[inline]
pub fn b2_collide_capsule_and_circle(capsuleA &B2Capsule, xfA B2Transform, circleB &B2Circle, xfB B2Transform) B2Manifold  {
	return C.b2CollideCapsuleAndCircle(capsuleA, xfA, circleB, xfB)
}

/// Compute the contact manifold between an segment and a circle
fn C.b2CollideSegmentAndCircle(segmentA &B2Segment, xfA B2Transform, circleB &B2Circle, xfB B2Transform) B2Manifold 
@[inline]
pub fn b2_collide_segment_and_circle(segmentA &B2Segment, xfA B2Transform, circleB &B2Circle, xfB B2Transform) B2Manifold  {
	return C.b2CollideSegmentAndCircle(segmentA, xfA, circleB, xfB)
}

/// Compute the contact manifold between a polygon and a circle
fn C.b2CollidePolygonAndCircle(polygonA &B2Polygon, xfA B2Transform, circleB &B2Circle, xfB B2Transform) B2Manifold 
@[inline]
pub fn b2_collide_polygon_and_circle(polygonA &B2Polygon, xfA B2Transform, circleB &B2Circle, xfB B2Transform) B2Manifold  {
	return C.b2CollidePolygonAndCircle(polygonA, xfA, circleB, xfB)
}

/// Compute the contact manifold between a capsule and circle
fn C.b2CollideCapsules(capsuleA &B2Capsule, xfA B2Transform, capsuleB &B2Capsule, xfB B2Transform) B2Manifold 
@[inline]
pub fn b2_collide_capsules(capsuleA &B2Capsule, xfA B2Transform, capsuleB &B2Capsule, xfB B2Transform) B2Manifold  {
	return C.b2CollideCapsules(capsuleA, xfA, capsuleB, xfB)
}

/// Compute the contact manifold between an segment and a capsule
fn C.b2CollideSegmentAndCapsule(segmentA &B2Segment, xfA B2Transform, capsuleB &B2Capsule, xfB B2Transform) B2Manifold 
@[inline]
pub fn b2_collide_segment_and_capsule(segmentA &B2Segment, xfA B2Transform, capsuleB &B2Capsule, xfB B2Transform) B2Manifold  {
	return C.b2CollideSegmentAndCapsule(segmentA, xfA, capsuleB, xfB)
}

/// Compute the contact manifold between a polygon and capsule
fn C.b2CollidePolygonAndCapsule(polygonA &B2Polygon, xfA B2Transform, capsuleB &B2Capsule, xfB B2Transform) B2Manifold 
@[inline]
pub fn b2_collide_polygon_and_capsule(polygonA &B2Polygon, xfA B2Transform, capsuleB &B2Capsule, xfB B2Transform) B2Manifold  {
	return C.b2CollidePolygonAndCapsule(polygonA, xfA, capsuleB, xfB)
}

/// Compute the contact manifold between two polygons
fn C.b2CollidePolygons(polygonA &B2Polygon, xfA B2Transform, polygonB &B2Polygon, xfB B2Transform) B2Manifold 
@[inline]
pub fn b2_collide_polygons(polygonA &B2Polygon, xfA B2Transform, polygonB &B2Polygon, xfB B2Transform) B2Manifold  {
	return C.b2CollidePolygons(polygonA, xfA, polygonB, xfB)
}

/// Compute the contact manifold between an segment and a polygon
fn C.b2CollideSegmentAndPolygon(segmentA &B2Segment, xfA B2Transform, polygonB &B2Polygon, xfB B2Transform) B2Manifold 
@[inline]
pub fn b2_collide_segment_and_polygon(segmentA &B2Segment, xfA B2Transform, polygonB &B2Polygon, xfB B2Transform) B2Manifold  {
	return C.b2CollideSegmentAndPolygon(segmentA, xfA, polygonB, xfB)
}

/// Compute the contact manifold between a smooth segment and a circle
fn C.b2CollideSmoothSegmentAndCircle(smoothSegmentA &B2SmoothSegment, xfA B2Transform, circleB &B2Circle, xfB B2Transform) B2Manifold
@[inline]
pub fn b2_collide_smooth_segment_and_circle(smoothSegmentA &B2SmoothSegment, xfA B2Transform, circleB &B2Circle, xfB B2Transform) B2Manifold  {
	return C.b2CollideSmoothSegmentAndCircle(smoothSegmentA, xfA, circleB, xfB)
}

/// Compute the contact manifold between an segment and a capsule
fn C.b2CollideSmoothSegmentAndCapsule(smoothSegmentA &B2SmoothSegment, xfA B2Transform, capsuleB &B2Capsule, xfB B2Transform, cache &B2DistanceCache) B2Manifold 
@[inline]
pub fn b2_collide_smooth_segment_and_capsule(smoothSegmentA &B2SmoothSegment, xfA B2Transform, capsuleB &B2Capsule, xfB B2Transform, cache &B2DistanceCache) B2Manifold  {
	return C.b2CollideSmoothSegmentAndCapsule(smoothSegmentA, xfA, capsuleB, xfB, cache)
}

/// Compute the contact manifold between a smooth segment and a rounded polygon
fn C.b2CollideSmoothSegmentAndPolygon(smoothSegmentA &B2SmoothSegment, xfA B2Transform, polygonB &B2Polygon, xfB B2Transform, cache &B2DistanceCache) B2Manifold 
@[inline]
pub fn b2_collide_smooth_segment_and_polygon(smoothSegmentA &B2SmoothSegment, xfA B2Transform, polygonB &B2Polygon, xfB B2Transform, cache &B2DistanceCache) B2Manifold  {
	return C.b2CollideSmoothSegmentAndPolygon(smoothSegmentA, xfA, polygonB, xfB, cache)
}

/// Constructing the tree initializes the node pool.
fn C.b2DynamicTree_Create() B2DynamicTree
@[inline]
pub fn b2_dynamic_tree_create() B2DynamicTree {
	return C.b2DynamicTree_Create()
}

/// Destroy the tree, freeing the node pool.
fn C.b2DynamicTree_Destroy(tree &B2DynamicTree)
@[inline]
pub fn b2_dynamic_tree_destroy(tree &B2DynamicTree) {
	C.b2DynamicTree_Destroy(tree)
}

/// Create a proxy. Provide an AABB and a userData value.
fn C.b2DynamicTree_CreateProxy(tree &B2DynamicTree, aabb B2AABB, categoryBits u32, userData int) int
@[inline]
pub fn b2_dynamic_tree_create_proxy(tree &B2DynamicTree, aabb B2AABB, categoryBits u32, userData int) int {
	return C.b2DynamicTree_CreateProxy(tree, aabb, categoryBits, userData)
}

/// Destroy a proxy. This asserts if the id is invalid.
fn C.b2DynamicTree_DestroyProxy(tree &B2DynamicTree, proxyId int)
@[inline]
pub fn b2_dynamic_tree_destroy_proxy(tree &B2DynamicTree, proxyId int) {
	C.b2DynamicTree_DestroyProxy(tree, proxyId)
}

/// Move a proxy to a new AABB by removing and reinserting into the tree.
fn C.b2DynamicTree_MoveProxy(tree &B2DynamicTree, proxyId int, aabb B2AABB)
@[inline]
pub fn b2_dynamic_tree_move_proxy(tree &B2DynamicTree, proxyId int, aabb B2AABB) {
	C.b2DynamicTree_MoveProxy(tree, proxyId, aabb)
}

/// Enlarge a proxy and enlarge ancestors as necessary.
fn C.b2DynamicTree_EnlargeProxy(tree &B2DynamicTree, proxyId int, aabb B2AABB)
@[inline]
pub fn b2_dynamic_tree_enlarge_proxy(tree &B2DynamicTree, proxyId int, aabb B2AABB) {
	C.b2DynamicTree_EnlargeProxy(tree, proxyId, aabb)
}

/// Query an AABB for overlapping proxies. The callback class
/// is called for each proxy that overlaps the supplied AABB.
fn C.b2DynamicTree_Query(tree &B2DynamicTree, aabb B2AABB, maskBits u32, callback &B2TreeQueryCallbackFcn, context voidptr)
@[inline]
pub fn b2_dynamic_tree_query(tree &B2DynamicTree, aabb B2AABB, maskBits u32, callback &B2TreeQueryCallbackFcn, context voidptr) {
	C.b2DynamicTree_Query(tree, aabb, maskBits, callback, context)
}

/// Ray-cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray-cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
///	Bit-wise filtering using mask bits can greatly improve performance in some scenarios.
///	@param tree the dynamic tree to ray cast
/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1)
///	@param maskBits filter bits: `bool accept = (maskBits & node->categoryBits) != 0;`
/// @param callback a callback class that is called for each proxy that is hit by the ray
///	@param context user context that is passed to the callback
fn C.b2DynamicTree_RayCast(tree &B2DynamicTree, input &B2RayCastInput, maskBits u32, callback &B2TreeRayCastCallbackFcn, context voidptr)
@[inline]
pub fn b2_dynamic_tree_ray_cast(tree &B2DynamicTree, input &B2RayCastInput, maskBits u32, callback &B2TreeRayCastCallbackFcn, context voidptr) {
	C.b2DynamicTree_RayCast(tree, input, maskBits, callback, context)
}

/// Ray-cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray-cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
///	@param tree the dynamic tree to ray cast
/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
///	@param maskBits filter bits: `bool accept = (maskBits & node->categoryBits) != 0;`
/// @param callback a callback class that is called for each proxy that is hit by the shape
///	@param context user context that is passed to the callback
fn C.b2DynamicTree_ShapeCast(tree &B2DynamicTree, input &B2ShapeCastInput, maskBits u32, callback &B2TreeShapeCastCallbackFcn, context voidptr)
@[inline]
pub fn b2_dynamic_tree_shape_cast(tree &B2DynamicTree, input &B2ShapeCastInput, maskBits u32, callback &B2TreeShapeCastCallbackFcn, context voidptr) {
	C.b2DynamicTree_ShapeCast(tree, input, maskBits, callback, context)
}

/// Validate this tree. For testing.
fn C.b2DynamicTree_Validate(tree &B2DynamicTree)
@[inline]
pub fn b2_dynamic_tree_validate(tree &B2DynamicTree) {
	C.b2DynamicTree_Validate(tree)
}

/// Compute the height of the binary tree in O(N) time. Should not be
/// called often.
fn C.b2DynamicTree_GetHeight(tree &B2DynamicTree) int
@[inline]
pub fn b2_dynamic_tree_get_height(tree &B2DynamicTree) int {
	return C.b2DynamicTree_GetHeight(tree)
}

/// Get the maximum balance of the tree. The balance is the difference in height of the two children of a node.
fn C.b2DynamicTree_GetMaxBalance(tree &B2DynamicTree) int
@[inline]
pub fn b2_dynamic_tree_get_max_balance(tree &B2DynamicTree) int {
	return C.b2DynamicTree_GetMaxBalance(tree)
}

/// Get the ratio of the sum of the node areas to the root area.
fn C.b2DynamicTree_GetAreaRatio(tree &B2DynamicTree) f32
@[inline]
pub fn b2_dynamic_tree_get_area_ratio(tree &B2DynamicTree) f32 {
	return C.b2DynamicTree_GetAreaRatio(tree)
}

/// Build an optimal tree. Very expensive. For testing.
fn C.b2DynamicTree_RebuildBottomUp(tree &B2DynamicTree)
@[inline]
pub fn b2_dynamic_tree_rebuild_bottom_up(tree &B2DynamicTree) {
	C.b2DynamicTree_RebuildBottomUp(tree)
}

/// Get the number of proxies created
fn C.b2DynamicTree_GetProxyCount(tree &B2DynamicTree) int
@[inline]
pub fn b2_dynamic_tree_get_proxy_count(tree &B2DynamicTree) int {
	return C.b2DynamicTree_GetProxyCount(tree)
}

/// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
fn C.b2DynamicTree_Rebuild(tree &B2DynamicTree, fullBuild bool) int
@[inline]
pub fn b2_dynamic_tree_rebuild(tree &B2DynamicTree, fullBuild bool) int {
	return C.b2DynamicTree_Rebuild(tree, fullBuild)
}

/// Shift the world origin. Useful for large worlds.
/// The shift formula is: position -= newOrigin
/// @param tree the tree to shift
/// @param newOrigin the new origin with respect to the old origin
fn C.b2DynamicTree_ShiftOrigin(tree &B2DynamicTree, newOrigin B2Vec2)
@[inline]
pub fn b2_dynamic_tree_shift_origin(tree &B2DynamicTree, newOrigin B2Vec2) {
	C.b2DynamicTree_ShiftOrigin(tree, newOrigin)
}

/// Get the number of bytes used by this tree 
fn C.b2DynamicTree_GetByteCount(tree &B2DynamicTree) int
@[inline]
pub fn b2_dynamic_tree_get_byte_count(tree &B2DynamicTree) int {
	return C.b2DynamicTree_GetByteCount(tree)
}

/// Get proxy user data
/// @return the proxy user data or 0 if the id is invalid
fn C.b2DynamicTree_GetUserData(tree &B2DynamicTree, proxyId int) int
@[inline]
pub fn b2_dynamic_tree_get_user_data(tree &B2DynamicTree, proxyId int) int {
	return C.b2DynamicTree_GetUserData(tree, proxyId)
}

/// Get the AABB of a proxy
fn C.b2DynamicTree_GetAABB(tree &B2DynamicTree , proxyId int) B2AABB
@[inline]
pub fn b2_dynamic_tree_get_aabb(tree &B2DynamicTree , proxyId int) B2AABB {
	return C.b2DynamicTree_GetAABB(tree, proxyId)
}

// end collision

// start types 

type B2TaskCallback = fn(int, int, u32, voidptr) 
type B2EnqueueTaskCallback = fn(&B2TaskCallback, int, int, voidptr, voidptr) voidptr
type B2FinishTaskCallback = fn(voidptr, voidptr)

/// Result from b2World_RayCastClosest
/// ingroup world
@[typedef]
struct C.b2RayResult {
pub mut:
	shapeId B2ShapeId 
	point B2Vec2 
	normal B2Vec2 
	fraction f32
	hit bool
}

pub type B2RayResult = C.b2RayResult

/// World definition used to create a simulation world.
/// Must be initialized using b2DefaultWorldDef().
/// ingroup world
@[typedef] 
struct C.b2WorldDef {
pub mut:
	/// Gravity vector. Box2D has no up-vector defined.
	gravity B2Vec2 

	/// Restitution velocity threshold, usually in m/s. Collisions above this
	/// speed have restitution applied (will bounce).
	restitutionThreshold f32

	/// This parameter controls how fast overlap is resolved and has units of meters per second
	contactPushoutVelocity f32

	/// Threshold velocity for hit events. Usually meters per second.
	hitEventThreshold f32

	/// Contact stiffness. Cycles per second.
	contactHertz f32

	/// Contact bounciness. Non-dimensional.
	contactDampingRatio f32

	/// Joint stiffness. Cycles per second.
	jointHertz f32

	/// Joint bounciness. Non-dimensional.
	jointDampingRatio f32

	/// Can bodies go to sleep to improve performance
	enableSleep bool

	/// Enable continuous collision
	enableContinous bool

	/// Number of workers to use with the provided task system. Box2D performs best when using only
	///	performance cores and accessing a single L2 cache. Efficiency cores and hyper-threading provide
	///	little benefit and may even harm performance.
	workerCount int

	/// Function to spawn tasks
	enqueueTask &B2EnqueueTaskCallback 

	/// Function to finish a task
	finishTask &B2FinishTaskCallback 

	/// User context that is provided to enqueueTask and finishTask
	userTaskContext voidptr

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type B2WorldDef = C.b2WorldDef

/// The body simulation type.
/// Each body is one of these three types. The type determines how the body behaves in the simulation.
/// @ingroup body
enum B2BodyType {
	/// zero mass, zero velocity, may be manually moved
	b2_static_body = 0

	/// zero mass, velocity set by user, moved by solver
	b2_kinematic_body = 1

	/// positive mass, velocity determined by forces, moved by solver
	b2_dynamic_body = 2

	/// number of body types
	b2_body_type_count = 3
}

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
///	Body definitions are temporary objects used to bundle creation parameters.
/// Must be initialized using b2DefaultBodyDef().
/// @ingroup body
@[typedef] 
struct C.b2BodyDef {
pub mut:
	/// The body type: static, kinematic, or dynamic.
	type0 B2BodyType 

	/// The initial world position of the body. Bodies should be created with the desired position.
	/// @note Creating bodies at the origin and then moving them nearly doubles the cost of body creation, especially
	///	if the body is moved after shapes have been added.
	position B2Vec2 

	/// The initial world angle of the body in radians.
	rotation B2Rot

	/// The initial linear velocity of the body's origin. Typically in meters per second.
	linearVelocity B2Vec2

	/// The initial angular velocity of the body. Radians per second.
	angularVelocity f32

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	///	Generally linear damping is undesirable because it makes objects move slowly
	///	as if they are floating.
	linearDamping f32

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	///	Angular damping can be use slow down rotating bodies.
	angularDamping f32

	/// Scale the gravity applied to this body. Non-dimensional.
	gravityScale f32

	/// Sleep velocity threshold, default is 0.05 meter per second
	sleepThreshold f32

	/// Use this to store application specific body data.
	userData voidptr

	/// Set this flag to false if this body should never fall asleep.
	enableSleep bool

	/// Is this body initially awake or sleeping?
	isAwake bool

	/// Should this body be prevented from rotating? Useful for characters.
	fixedRotation bool

	/// Treat this body as high speed object that performs continuous collision detection
	/// against dynamic and kinematic bodies, but not other bullet bodies.
	///	@warning Bullets should be used sparingly. They are not a solution for general dynamic-versus-dynamic
	///	continuous collision. They may interfere with joint constraints.
	isBullet bool

	/// Used to disable a body. A disabled body does not move or collide.
	isEnabled bool

	/// Automatically compute mass and related properties on this body from shapes.
	/// Triggers whenever a shape is add/removed/changed. Default is true.
	automaticMass bool

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type B2BodyDef = C.b2BodyDef

/// This is used to filter collision on shapes. It affects shape-vs-shape collision
///	and shape-versus-query collision (such as b2World_CastRay).
/// @ingroup shape
@[typedef] 
struct C.b2Filter {
pub mut:
	/// The collision category bits. Normally you would just set one bit. The category bits should
	///	represent your application object types. For example:
	///	@code{.cpp}
	///	enum MyCategories
	///	{
	///	   Static  = 0x00000001,
	///	   Dynamic = 0x00000002,
	///	   Debris  = 0x00000004,
	///	   Player  = 0x00000008,
	///	   // etc
	/// }
	///	@endcode
	categoryBits u32

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	///	For example, you may want your player to only collide with static objects
	///	and other players.
	///	@code{.c}
	///	maskBits = Static | Player
	///	@endcode
	maskBits u32

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
	/// always wins against the mask bits.
	///	For example, you may want ragdolls to collide with other ragdolls but you don't want
	///	ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
	///	and apply that group index to all shapes on the ragdoll.
	groupIndex int
}

pub type B2Filter = C.b2Filter

/// The query filter is used to filter collisions between queries and shapes. For example,
///	you may want a ray-cast representing a projectile to hit players and the static environment
///	but not debris.
/// @ingroup shape
@[ypedef] 
struct C.b2QueryFilter {
	/// The collision category bits of this query. Normally you would just set one bit.
	categoryBits u32

	/// The collision mask bits. This states the shape categories that this
	/// query would accept for collision.
	maskBits u32
}

pub type B2QueryFilter = C.b2QueryFilter

/// Shape type
/// @ingroup shape
enum B2ShapeType {
	/// A circle with an offset
	b2_circle_shape

	/// A capsule is an extruded circle
	b2_capsule_shape

	/// A line segment
	b2_segment_shape

	/// A convex polygon
	b2_polygon_shape

	/// A smooth segment owned by a chain shape
	b2_smooth_segment_shape

	/// The number of shape types
	b2_shape_type_count
}

/// Used to create a shape.
/// This is a temporary object used to bundle shape creation parameters. You may use
///	the same shape definition to create multiple shapes.
/// Must be initialized using b2DefaultShapeDef().
/// @ingroup shape
@[typedef] 
struct C.b2ShapeDef {
pub mut:
	/// Use this to store application specific shape data.
	userData voidptr

	/// The Coulomb (dry) friction coefficient, usually in the range [0,1].
	friction f32

	/// The restitution (bounce) usually in the range [0,1].
	restitution f32

	/// The density, usually in kg/m^2.
	density f32

	/// Collision filtering data.
	filter B2Filter

	/// A sensor shape generates overlap events but never generates a collision response.
	isSensor bool

	/// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	enableSensorEvents bool

	/// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	enableContactEvents bool

	/// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	enableHitEvents bool

	/// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
	///	and must be carefully handled due to threading. Ignored for sensors.
	enablePreSolveEvents bool

	/// Normally shapes on static bodies don't invoke contact creation when they are added to the world. This overrides
	///	that behavior and causes contact creation. This significantly slows down static body creation which can be important
	///	when there are many static shapes.
	forceContactCreation bool

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type B2ShapeDef = C.b2ShapeDef

/// Used to create a chain of edges. This is designed to eliminate ghost collisions with some limitations.
///	- chains are one-sided
///	- chains have no mass and should be used on static bodies
///	- chains have a counter-clockwise winding order
///	- chains are either a loop or open
/// - a chain must have at least 4 points
///	- the distance between any two points must be greater than b2_linearSlop
///	- a chain shape should not self intersect (this is not validated)
///	- an open chain shape has NO COLLISION on the first and final edge
///	- you may overlap two open chains on their first three and/or last three points to get smooth collision
///	- a chain shape creates multiple smooth edges shapes on the body
/// https://en.wikipedia.org/wiki/Polygonal_chain
/// Must be initialized using b2DefaultChainDef().
///	@warning Do not use chain shapes unless you understand the limitations. This is an advanced feature.
/// @ingroup shape
@[typedef] 
struct C.b2ChainDef {
pub mut:
	/// Use this to store application specific shape data.
	userData voidptr

	/// An array of at least 4 points. These are cloned and may be temporary.
	points &B2Vec2

	/// The point count, must be 4 or more.
	count int

	/// The friction coefficient, usually in the range [0,1].
	friction f32

	/// The restitution (elasticity) usually in the range [0,1].
	restitution f32

	/// Contact filtering data.
	filter B2Filter 

	/// Indicates a closed chain formed by connecting the first and last points
	isLoop bool

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type B2ChainDef = C.b2ChainDef

//! @cond
/// Profiling data. Times are in milliseconds.
@[typedef] 
struct C.b2Profile {
pub mut:
	step f32
	pairs f32
	collide f32
	solve f32
	buildIslands f32
	solveConstraints f32
	prepareTasks f32
	solverTasks f32
	prepareConstraints f32
	integrateVelocities f32
	warmStart f32
	solveVelocities f32
	integratePositions f32
	relaxVelocities f32
	applyRestitution f32
	storeImpulses f32
	finalizeBodies f32
	splitIslands f32
	sleepIslands f32
	hitEvents f32
	broadphase f32
	continuous f32
}

pub type B2Profile = C.b2Profile

/// Counters that give details of the simulation size.
@[typedef] 
struct C.b2Counters {
pub mut:
	staticBodyCount int
	bodyCount int
	shapeCount int
	contactCount int
	jointCount int
	islandCount int
	stackUsed int
	staticTreeHeight int
	treeHeight int
	byteCount int
	taskCount int
	colorCounts [12]int
}

pub type B2Counters = C.b2Counters

enum B2JointType {
	b2_distance_joint
	b2_motor_joint
	b2_mouse_joint
	b2_prismatic_join
	b2_revolute_joint
	b2_weld_joint
	b2_wheel_joint
} 

/// Distance joint definition
///
/// This requires defining an anchor point on both
/// bodies and the non-zero distance of the distance joint. The definition uses
/// local anchor points so that the initial configuration can violate the
/// constraint slightly. This helps when saving and loading a game.
/// @ingroup distance_joint
@[typedef] 
struct C.b2DistanceJointDef {
pub mut:	
	/// The first attached body
	bodyIdA B2BodyId

	/// The second attached body
	bodyIdB B2BodyId

	/// The local anchor point relative to bodyA's origin
	localAnchorA B2Vec2 

	/// The local anchor point relative to bodyB's origin
	localAnchorB B2Vec2

	/// The rest length of this joint. Clamped to a stable minimum value.
	length f32

	/// Enable the distance constraint to behave like a spring. If false
	///	then the distance joint will be rigid, overriding the limit and motor.
	enableSpring bool

	/// The spring linear stiffness Hertz, cycles per second
	hertz f32

	/// The spring linear damping ratio, non-dimensional
	dampingRatio f32

	/// Enable/disable the joint limit
	enableLimit bool

	/// Minimum length. Clamped to a stable minimum value.
	minLength f32

	/// Maximum length. Must be greater than or equal to the minimum length.
	maxLength f32

	/// Enable/disable the joint motor
	enableMotor bool

	/// The maximum motor force, usually in newtons
	maxMotorForce f32

	/// The desired motor speed, usually in meters per second
	motorSpeed f32

	/// Set this flag to true if the attached bodies should collide
	collideConnected bool

	/// User data pointer
	userData voidptr

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type B2DistanceJointDef = C.b2DistanceJointDef

/// A motor joint is used to control the relative motion between two bodies
///
/// A typical usage is to control the movement of a dynamic body with respect to the ground.
/// @ingroup motor_joint
@[typedef] 
struct C.b2MotorJointDef {
pub mut:
	/// The first attached body
	bodyIdA B2BodyId 

	/// The second attached body
	bodyIdB B2BodyId

	/// Position of bodyB minus the position of bodyA, in bodyA's frame
	linearOffset B2Vec2

	/// The bodyB angle minus bodyA angle in radians
	angularOffset f32

	/// The maximum motor force in newtons
	maxForce f32

	/// The maximum motor torque in newton-meters
	maxTorque f32

	/// Position correction factor in the range [0,1]
	correctionFactor f32

	/// Set this flag to true if the attached bodies should collide
	collideConnected bool

	/// User data pointer
	userData voidptr

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type B2MotorJointDef = C.b2MotorJointDef

/// A mouse joint is used to make a point on a body track a specified world point.
///
/// This a soft constraint and allows the constraint to stretch without
/// applying huge forces. This also applies rotation constraint heuristic to improve control.
/// @ingroup mouse_joint
@[typedef] 
struct C.b2MouseJointDef {
	/// The first attached body.
	bodyIdA B2BodyId 

	/// The second attached body.
	bodyIdB B2BodyId

	/// The initial target point in world space
	target B2Vec2 

	/// Stiffness in hertz
	hertz f32

	/// Damping ratio, non-dimensional
	dampingRatio f32

	/// Maximum force, typically in newtons
	maxForce f32

	/// Set this flag to true if the attached bodies should collide.
	collideConnected bool

	/// User data pointer
	userData voidptr

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type B2MouseJointDef = C.b2MouseJointDef

/// Prismatic joint definition
///
/// This requires defining a line of motion using an axis and an anchor point.
/// The definition uses local anchor points and a local axis so that the initial
/// configuration can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space.
@[typedef] 
struct C.b2PrismaticJointDef {
pub mut:	
	/// The first attached body
	bodyIdA B2BodyId 

	/// The second attached body
	bodyIdB B2BodyId

	/// The local anchor point relative to bodyA's origin
	localAnchorA B2Vec2

	/// The local anchor point relative to bodyB's origin
	localAnchorB B2Vec2

	/// The local translation unit axis in bodyA
	localAxisA B2Vec2

	/// The constrained angle between the bodies: bodyB_angle - bodyA_angle
	referenceAngle f32

	/// Enable a linear spring along the prismatic joint axis
	enableSpring bool

	/// The spring stiffness Hertz, cycles per second
	hertz f32

	/// The spring damping ratio, non-dimensional
	dampingRatio f32

	/// Enable/disable the joint limit
	enableLimit bool

	/// The lower translation limit
	lowerTranslation f32

	/// The upper translation limit
	upperTranslation f32

	/// Enable/disable the joint motor
	enableMotor bool

	/// The maximum motor force, typically in newtons
	maxMotorForce f32

	/// The desired motor speed, typically in meters per second
	motorSpeed f32

	/// Set this flag to true if the attached bodies should collide
	collideConnected bool

	/// User data pointer
	userData voidptr

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type B2PrismaticJointDef = C.b2PrismaticJointDef

/// Revolute joint definition
///
/// This requires defining an anchor point where the bodies are joined.
/// The definition uses local anchor points so that the
/// initial configuration can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This helps when saving
/// and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be
/// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken
/// @ingroup revolute_joint
@[typedef] 
struct C.b2RevoluteJointDef {
pub mut:
	/// The first attached body
	bodyIdA B2BodyId 

	/// The second attached body
	bodyIdB B2BodyId

	/// The local anchor point relative to bodyA's origin
	localAnchorA B2Vec2

	/// The local anchor point relative to bodyB's origin
	localAnchorB B2Vec2

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	/// This defines the zero angle for the joint limit.
	referenceAngle f32

	/// Enable a rotational spring on the revolute hinge axis
	enableSpring bool

	/// The spring stiffness Hertz, cycles per second
	hertz f32

	/// The spring damping ratio, non-dimensional
	dampingRatio f32

	/// A flag to enable joint limits
	enableLimit bool

	/// The lower angle for the joint limit in radians
	lowerAngle f32

	/// The upper angle for the joint limit in radians
	upperAngle f32

	/// A flag to enable the joint motor
	enableMotor bool

	/// The maximum motor torque, typically in newton-meters
	maxMotorTorque f32

	/// The desired motor speed in radians per second
	motorSpeed f32

	/// Scale the debug draw
	drawSize f32

	/// Set this flag to true if the attached bodies should collide
	collideConnected bool

	/// User data pointer
	userData voidptr

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type B2RevoluteJointDef = C.b2RevoluteJointDef

/// Weld joint definition
///
/// A weld joint connect to bodies together rigidly. This constraint provides springs to mimic
///	soft-body simulation.
/// @note The approximate solver in Box2D cannot hold many bodies together rigidly
/// @ingroup weld_joint
@[typedef] 
struct C.b2WeldJointDef {
pub mut:
	/// The first attached body
	bodyIdA B2BodyId 

	/// The second attached body
	bodyIdB B2BodyId 

	/// The local anchor point relative to bodyA's origin
	localAnchorA B2Vec2 

	/// The local anchor point relative to bodyB's origin
	localAnchorB B2Vec2

	/// The bodyB angle minus bodyA angle in the reference state (radians)
	referenceAngle f32

	/// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
	linearHertz f32

	/// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
	angularHertz f32

	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	linearDampingRatio f32

	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	angularDampingRatio f32

	/// Set this flag to true if the attached bodies should collide
	collideConnected bool

	/// User data pointer
	userData voidptr

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type B2WeldJointDef = C.b2WeldJointDef

/// Wheel joint definition
///
/// This requires defining a line of motion using an axis and an anchor point.
/// The definition uses local  anchor points and a local axis so that the initial
/// configuration can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space.
/// @ingroup wheel_joint
@[typedef] 
struct C.b2WheelJointDef {
pub mut:
	/// The first attached body
	bodyIdA B2BodyId 

	/// The second attached body
	bodyIdB B2BodyId

	/// The local anchor point relative to bodyA's origin
	localAnchorA B2Vec2 

	/// The local anchor point relative to bodyB's origin
	localAnchorB B2Vec2 

	/// The local translation unit axis in bodyA
	localAxisA B2Vec2 

	/// Enable a linear spring along the local axis
	enableSpring bool

	/// Spring stiffness in Hertz
	hertz f32

	/// Spring damping ratio, non-dimensional
	dampingRatio f32

	/// Enable/disable the joint linear limit
	enableLimit bool

	/// The lower translation limit
	lowerTranslation f32

	/// The upper translation limit
	upperTranslation f32

	/// Enable/disable the joint rotational motor
	enableMotor bool

	/// The maximum motor torque, typically in newton-meters
	maxMotorTorquef f32

	/// The desired motor speed in radians per second
	motorSpeed f32

	/// Set this flag to true if the attached bodies should collide
	collideConnected bool

	/// User data pointer
	userData voidptr

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type B2WheelJointDef = C.b2WheelJointDef

// A begin touch event is generated when a shape starts to overlap a sensor shape.
@[typedef] 
struct C.b2SensorBeginTouchEvent {
pub mut:
	/// The id of the sensor shape
	sensorShapeId B2ShapeId

	/// The id of the dynamic shape that began touching the sensor shape
	visitorShapeId B2ShapeId 
}

pub type B2SensorBeginTouchEvent = C.b2SensorBeginTouchEvent

/// An end touch event is generated when a shape stops overlapping a sensor shape.
@[typedef] 
struct C.b2SensorEndTouchEvent {
pub mut:
	/// The id of the sensor shape
	sensorShapeId B2ShapeId 

	/// The id of the dynamic shape that stopped touching the sensor shape
	visitorShapeId B2ShapeId
}

pub type B2SensorEndTouchEvent = C.b2SensorEndTouchEvent

/// Sensor events are buffered in the Box2D world and are available
///	as begin/end overlap event arrays after the time step is complete.
///	Note: these may become invalid if bodies and/or shapes are destroyed
@[typedef] 
struct C.b2SensorEvents {
pub mut:
	/// Array of sensor begin touch events
	 beginEvents &B2SensorBeginTouchEvent

	/// Array of sensor end touch events
	endEvents &B2SensorBeginTouchEvent

	/// The number of begin touch events
	beginCount int

	/// The number of end touch events
	endCount int
}

pub type B2SensorEvents = C.b2SensorEvents

/// A begin touch event is generated when two shapes begin touching.
@[typedef] 
struct C.b2ContactBeginTouchEvent {
pub mut:
	/// Id of the first shape
	shapeIdA B2ShapeId 

	/// Id of the second shape
	shapeIdB B2ShapeId
}

pub type B2ContactBeginTouchEvent = C.b2ContactBeginTouchEvent 

/// An end touch event is generated when two shapes stop touching.
@[typedef] 
struct C.b2ContactEndTouchEvent {
pub mut:
	/// Id of the first shape
	shapeIdA B2ShapeId 

	/// Id of the second shape
	shapeIdB B2ShapeId
}

pub type B2ContactEndTouchEvent = C.b2ContactEndTouchEvent

/// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
@[typedef] 
struct C.b2ContactHitEvent {
pub mut:
	/// Id of the first shape
	shapeIdA B2ShapeId

	/// Id of the second shape
	shapeIdB B2ShapeId

	/// Point where the shapes hit
	point B2Vec2

	/// Normal vector pointing from shape A to shape B
	normal B2Vec2

	/// The speed the shapes are approaching. Always positive. Typically in meters per second.
	approachSpeed f32
}

pub type B2ContactHitEvent = C.b2ContactHitEvent

/// Contact events are buffered in the Box2D world and are available
///	as event arrays after the time step is complete.
///	Note: these may become invalid if bodies and/or shapes are destroyed
@[typedef] 
struct C.b2ContactEvents {
pub mut:
	/// Array of begin touch events
	beginEvents &B2ContactBeginTouchEvent

	/// Array of end touch events
	endEvents &B2ContactEndTouchEvent

	/// Array of hit events
	hitEvents &B2ContactHitEvent

	/// Number of begin touch events
	beginCount int

	/// Number of end touch events
	endCount int

	/// Number of hit events
	hitCount int
}

pub type B2ContactEvents = C.b2ContactEvents

/// Body move events triggered when a body moves.
/// Triggered when a body moves due to simulation. Not reported for bodies moved by the user.
/// This also has a flag to indicate that the body went to sleep so the application can also
/// sleep that actor/entity/object associated with the body.
/// On the other hand if the flag does not indicate the body went to sleep then the application
/// can treat the actor/entity/object associated with the body as awake.
///	This is an efficient way for an application to update game object transforms rather than
///	calling functions such as b2Body_GetTransform() because this data is delivered as a contiguous array
///	and it is only populated with bodies that have moved.
///	@note If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
@[typedef] 
struct C.b2BodyMoveEvent {
pub mut:
	transform B2Transform
	bodyId B2BodyId 
	userData voidptr
	fellAsleep bool
}

pub type B2BodyMoveEvent = C.b2BodyMoveEvent

/// Body events are buffered in the Box2D world and are available
///	as event arrays after the time step is complete.
///	Note: this date becomes invalid if bodies are destroyed
@[typedef] 
struct C.b2BodyEvents {
pub mut:
	/// Array of move events
	moveEvents &B2BodyMoveEvent

	/// Number of move events
	moveCount int
}

pub type B2BodyEvents = C.b2BodyEvents

/// The contact data for two shapes. By convention the manifold normal points
///	from shape A to shape B.
///	@see b2Shape_GetContactData() and b2Body_GetContactData()
@[typedef] 
struct C.b2ContactData {
	shapeIdA B2ShapeId
	shapeIdB B2ShapeId
	manifold B2Manifold 
}

pub type B2ContactData = C.b2ContactData

pub type B2CustomFilterFcn = fn(B2ShapeId, B2ShapeId, voidptr) bool
pub type B2PreSolveFcn = fn(B2ShapeId, B2ShapeId, B2Manifold, voidptr) bool
pub type B2OverlapResultFcn = fn(B2ShapeId, voidptr) bool
pub type B2CastResultFcn = fn(B2ShapeId, B2Vec2, B2Vec2, f32, voidptr) f32

/// These colors are used for debug draw.
enum B2HexColor {
	b2_color_alice_blue = 0xf0f8ff
	b2_color_antique_white = 0xfaebd7
	b2_color_aqua = 0x00ffff
	b2_color_aquamarine = 0x7fffd4
	b2_color_azure = 0xf0ffff
	b2_color_beige = 0xf5f5dc
	b2_color_bisque = 0xffe4c4
	b2_color_black = 0x000000
	b2_color_blanched_almond = 0xffebcd
	b2_color_blue = 0x0000ff
	b2_color_blue_violet = 0x8a2be2
	b2_color_brown = 0xa52a2a
	b2_color_burlywood = 0xdeb887
	b2_color_cadet_blue = 0x5f9ea0
	b2_color_chartreuse = 0x7fff00
	b2_color_chocolate = 0xd2691e
	b2_color_coral = 0xff7f50
	b2_color_cornflower_blue = 0x6495ed
	b2_color_cornsilk = 0xfff8dc
	b2_color_crimson = 0xdc143c
	// b2_color_cyan = 0x00ffff
	b2_color_dark_blue = 0x00008b
	b2_color_dark_cyan = 0x008b8b
	b2_color_dark_goldenrod = 0xb8860b
	b2_color_dark_gray = 0xa9a9a9
	b2_color_dark_green = 0x006400
	b2_color_dark_khaki = 0xbdb76b
	b2_color_dark_magenta = 0x8b008b
	b2_color_dark_olive_green = 0x556b2f
	b2_color_dark_orange = 0xff8c00
	b2_color_dark_orchid = 0x9932cc
	b2_color_dark_red = 0x8b0000
	b2_color_dark_salmon = 0xe9967a
	b2_color_dark_sea_green = 0x8fbc8f
	b2_color_dark_slate_blue = 0x483d8b
	b2_color_dark_slate_gray = 0x2f4f4f
	b2_color_dark_turquoise = 0x00ced1
	b2_color_dark_violet = 0x9400d3
	b2_color_deep_pink = 0xff1493
	b2_color_deep_sky_blue = 0x00bfff
	b2_color_dim_gray = 0x696969
	b2_color_dodger_blue = 0x1e90ff
	b2_color_firebrick = 0xb22222
	b2_color_floral_white = 0xfffaf0
	b2_color_forest_green = 0x228b22
	b2_color_fuchsia = 0xff00ff
	b2_color_gainsboro = 0xdcdcdc
	b2_color_ghost_white = 0xf8f8ff
	b2_color_gold = 0xffd700
	b2_color_goldenrod = 0xdaa520
	b2_color_gray = 0xbebebe
	b2_color_gray1 = 0x1a1a1a
	b2_color_gray2 = 0x333333
	b2_color_gray3 = 0x4d4d4d
	b2_color_gray4 = 0x666666
	b2_color_gray5 = 0x7f7f7f
	b2_color_gray6 = 0x999999
	b2_color_gray7 = 0xb3b3b3
	b2_color_gray8 = 0xcccccc
	b2_color_gray9 = 0xe5e5e5
	b2_color_green = 0x00ff00
	b2_color_green_yellow = 0xadff2f
	b2_color_honeydew = 0xf0fff0
	b2_color_hot_pink = 0xff69b4
	b2_color_indian_red = 0xcd5c5c
	b2_color_indigo = 0x4b0082
	b2_color_ivory = 0xfffff0
	b2_color_khaki = 0xf0e68c
	b2_color_lavender = 0xe6e6fa
	b2_color_lavender_blush = 0xfff0f5
	b2_color_lawn_green = 0x7cfc00
	b2_color_lemon_chiffon = 0xfffacd
	b2_color_light_blue = 0xadd8e6
	b2_color_light_coral = 0xf08080
	b2_color_light_cyan = 0xe0ffff
	b2_color_light_goldenrod = 0xeedd82
	b2_color_light_goldenrod_yellow = 0xfafad2
	b2_color_light_gray = 0xd3d3d3
	b2_color_light_green = 0x90ee90
	b2_color_light_pink = 0xffb6c1
	b2_color_light_salmon = 0xffa07a
	b2_color_light_sea_green = 0x20b2aa
	b2_color_light_sky_blue = 0x87cefa
	b2_color_light_slate_blue = 0x8470ff
	b2_color_light_slate_gray = 0x778899
	b2_color_light_steel_blue = 0xb0c4de
	b2_color_light_yellow = 0xffffe0
	// b2_color_lime = 0x00ff00
	b2_color_lime_green = 0x32cd32
	b2_color_linen = 0xfaf0e6
	// b2_color_magenta = 0xff00ff
	b2_color_maroon = 0xb03060
	b2_color_medium_aquamarine = 0x66cdaa
	b2_color_medium_blue = 0x0000cd
	b2_color_medium_orchid = 0xba55d3
	b2_color_medium_purple = 0x9370db
	b2_color_medium_sea_green = 0x3cb371
	b2_color_medium_slate_blue = 0x7b68ee
	b2_color_medium_spring_green = 0x00fa9a
	b2_color_medium_turquoise = 0x48d1cc
	b2_color_medium_violet_red = 0xc71585
	b2_color_midnight_blue = 0x191970
	b2_color_mint_cream = 0xf5fffa
	b2_color_misty_rose = 0xffe4e1
	b2_color_moccasin = 0xffe4b5
	b2_color_navajo_white = 0xffdead
	b2_color_navy = 0x000080
	// b2_color_navy_blue = 0x000080
	b2_color_old_lace = 0xfdf5e6
	b2_color_olive = 0x808000
	b2_color_olive_drab = 0x6b8e23
	b2_color_orange = 0xffa500
	b2_color_orange_red = 0xff4500
	b2_color_orchid = 0xda70d6
	b2_color_pale_goldenrod = 0xeee8aa
	b2_color_pale_green = 0x98fb98
	b2_color_pale_turquoise = 0xafeeee
	b2_color_pale_violet_red = 0xdb7093
	b2_color_papaya_whip = 0xffefd5
	b2_color_peach_puff = 0xffdab9
	b2_color_peru = 0xcd853f
	b2_color_pink = 0xffc0cb
	b2_color_plum = 0xdda0dd
	b2_color_powder_blue = 0xb0e0e6
	b2_color_purple = 0xa020f0
	b2_color_rebecca_purple = 0x663399
	b2_color_red = 0xff0000
	b2_color_rosy_brown = 0xbc8f8f
	b2_color_royal_blue = 0x4169e1
	b2_color_saddle_brown = 0x8b4513
	b2_color_salmon = 0xfa8072
	b2_color_sandy_brown = 0xf4a460
	b2_color_sea_green = 0x2e8b57
	b2_color_seashell = 0xfff5ee
	b2_color_sienna = 0xa0522d
	b2_color_silver = 0xc0c0c0
	b2_color_sky_blue = 0x87ceeb
	b2_color_slate_blue = 0x6a5acd
	b2_color_slate_gray = 0x708090
	b2_color_snow = 0xfffafa
	b2_color_spring_green = 0x00ff7f
	b2_color_steel_blue = 0x4682b4
	b2_color_tan = 0xd2b48c
	b2_color_teal = 0x008080
	b2_color_thistle = 0xd8bfd8
	b2_color_tomato = 0xff6347
	b2_color_turquoise = 0x40e0d0
	b2_color_violet = 0xee82ee
	b2_color_violet_red = 0xd02090
	b2_color_wheat = 0xf5deb3
	b2_color_white = 0xffffff
	b2_color_white_smoke = 0xf5f5f5
	b2_color_yellow = 0xffff00
	b2_color_yellow_green = 0x9acd32
}

/// This struct holds callbacks you can implement to draw a Box2D world.
///	@ingroup world
@[typedef] 
struct C.b2DebugDraw {
pub mut:
	/// Draw a closed polygon provided in CCW order.
	DrawPolygon fn(vertices B2Vec2, vertexCount int, color B2HexColor, context voidptr)

	/// Draw a solid closed polygon provided in CCW order.
	DrawSolidPolygon fn(transform B2Transform, vertices &B2Vec2, vertexCount int, radius f32, color B2HexColor ,context voidptr)

	/// Draw a circle.
	DrawCircle fn(center B2Vec2, radius f32, color  B2HexColor, context voidptr)

	/// Draw a solid circle.
	DrawSolidCircle fn(transform B2Transform , radius f32, color B2HexColor, context voidptr)

	/// Draw a capsule.
	DrawCapsule fn(p1 B2Vec2, p2 B2Vec2, radius f32, color B2HexColor, context voidptr)

	/// Draw a solid capsule.
	DrawSolidCapsule fn(p1 B2Vec2 , p2 B2Vec2 , radius f32, color B2HexColor, context voidptr)

	/// Draw a line segment.
	DrawSegment fn(p1 B2Vec2, p2 B2Vec2 , color B2HexColor , context voidptr)

	/// Draw a transform. Choose your own length scale.
	DrawTransform fn(transform B2Transform , context voidptr)

	/// Draw a point.
	DrawPoint fn(p B2Vec2, size f32, color B2HexColor, context voidptr)

	/// Draw a string.
	DrawString fn(p B2Vec2, s &char, context voidptr)

	/// Bounds to use if restricting drawing to a rectangular region
	drawingBounds B2AABB 

	/// Option to restrict drawing to a rectangular region. May suffer from unstable depth sorting.
	useDrawingBounds bool

	/// Option to draw shapes
	drawShapes bool

	/// Option to draw joints
	drawJoints bool

	/// Option to draw additional information for joints
	drawJointExtras bool

	/// Option to draw the bounding boxes for shapes
	drawAABBs bool

	/// Option to draw the mass and center of mass of dynamic bodies
	drawMass bool

	/// Option to draw contact points
	drawContacts bool

	/// Option to visualize the graph coloring used for contacts and joints
	drawGraphColors bool

	/// Option to draw contact normals
	drawContactNormals bool

	/// Option to draw contact normal impulses
	drawContactImpulses bool

	/// Option to draw contact friction impulses
	drawFrictionImpulses bool

	/// User context that is passed as an argument to drawing callback functions
	context voidptr
}

pub type B2DebugDraw = C.b2DebugDraw

/// Use this to initialize your world definition
/// @ingroup world
fn C.b2DefaultWorldDef() B2WorldDef
@[inline]
pub fn b2_default_world_def() B2WorldDef {
	return C.b2DefaultWorldDef()
}

/// Use this to initialize your body definition
/// @inbody
fn C.b2DefaultBodyDef() B2BodyDef
@[inline]
pub fn b2_default_body_def() B2BodyDef {
	return C.b2DefaultBodyDef()
}

/// Use this to initialize your filter
/// @ingroup shape
fn C.b2DefaultFilter() B2Filter
@[inline]
pub fn b2_default_filter() B2Filter {
	return C.b2DefaultFilter()
}

/// Use this to initialize your query filter
/// @ingroup shape
fn C.b2DefaultQueryFilter() B2QueryFilter
@[inline]
pub fn b2_default_query_filter() B2QueryFilter {
	return C.b2DefaultQueryFilter()
}

/// Use this to initialize your shape definition
/// @ingroup shape
fn C.b2DefaultShapeDef() B2ShapeDef
@[inline]
pub fn b2_default_shape_def() B2ShapeDef {
	return C.b2DefaultShapeDef()
}

// Use this to initialize your chain definition
/// @ingroup shape
fn C.b2DefaultChainDef() B2ChainDef
@[inline]
pub fn b2_default_chain_def() B2ChainDef {
	return C.b2DefaultChainDef()
}

/// Use this to initialize your joint definition
/// @ingroup distance_joint
fn C.b2DefaultDistanceJointDef() B2DistanceJointDef
@[inline]
pub fn b2_default_distance_joint_def() B2DistanceJointDef {
	return C.b2DefaultDistanceJointDef()
}

/// Use this to initialize your joint definition
/// @ingroup motor_joint
fn C.b2DefaultMotorJointDef() B2MotorJointDef
@[inline]
pub fn b2_default_motor_joint_def() B2MotorJointDef {
	return C.b2DefaultMotorJointDef()
}

/// Use this to initialize your joint definition
/// @ingroup mouse_joint
fn C.b2DefaultMouseJointDef() B2MouseJointDef
@[inline]
pub fn b2_default_mouse_joint_def() B2MouseJointDef {
	return C.b2DefaultMouseJointDef() 
}

/// Use this to initialize your joint definition
/// @ingroupd prismatic_joint
fn C.b2DefaultPrismaticJointDef() B2PrismaticJointDef
@[inline]
pub fn b2_default_prismatic_joint_def() B2PrismaticJointDef {
	return C.b2DefaultPrismaticJointDef()
}

/// Use this to initialize your joint definition.
/// @ingroup revolute_joint
fn C.b2DefaultRevoluteJointDef() B2RevoluteJointDef 
@[inline]
pub fn b2_default_revolute_joint_def() B2RevoluteJointDef {
	return C.b2DefaultRevoluteJointDef()
}

/// Use this to initialize your joint definition
/// @ingroup weld_joint
fn C.b2DefaultWeldJointDef() B2WeldJointDef
@[inline]
pub fn b2_default_weld_joint_def() B2WeldJointDef {
	return C.b2DefaultWeldJointDef()
}

/// Use this to initialize your joint definition
/// @ingroup wheel_joint
fn C.b2DefaultWheelJointDef() B2WheelJointDef 
@[inline]
pub fn b2_default_wheel_joint_def() B2WheelJointDef {
	return C.b2DefaultWheelJointDef()
}

// end types

// start bod2d

/// Create a world for rigid body simulation. A world contains bodies, shapes, and constraints. You make create
///	up to 128 worlds. Each world is completely independent and may be simulated in parallel.
///	@return the world id.
fn C.b2CreateWorld(def &B2WorldDef) B2WorldId 
@[inline]
pub fn b2_create_world(def &B2WorldDef) B2WorldId  {
	return C.b2CreateWorld(def) 
}











fn main() {
	// a := C.b2BodyDef{}
	// println(a)
	a := B2Vec2{2, 1}
	b := B2Vec2{3, 2}
	// println(a + b)
	println(b2_sub(b, a))
	 
}








































