// Unity C# reference source
// Copyright (c) Unity Technologies. For terms of use, see
// https://unity3d.com/legal/licenses/Unity_Reference_Only_License

using System;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Scripting;
using UnityEngine.Bindings;
using uei = UnityEngine.Internal;
using System.Globalization;
using System.Runtime.CompilerServices;


namespace SoftFloat
{
    static class MethodImplOptionsEx
    {
        public const short AggressiveInlining = 256;
    }
    // Representation of 2D vectors and points.
    [StructLayout(LayoutKind.Sequential)]
    // [NativeClass("sVector2f")]
    // [RequiredByNativeCode(Optional = true, GenerateProxy = true)]
    // [Unity.IL2CPP.CompilerServices.Il2CppEagerStaticClassConstruction]
    public struct sVector2 : IEquatable<sVector2>, IFormattable
    {
        // X component of the vector.
        public sfloat x;
        // Y component of the vector.
        public sfloat y;

        // Access the /x/ or /y/ component using [0] or [1] respectively.
        public sfloat this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0: return x;
                    case 1: return y;
                    default:
                        throw new IndexOutOfRangeException("Invalid sVector2 index!");
                }
            }

            set
            {
                switch (index)
                {
                    case 0: x = value; break;
                    case 1: y = value; break;
                    default:
                        throw new IndexOutOfRangeException("Invalid sVector2 index!");
                }
            }
        }

        // Constructs a new vector with given x, y components.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public sVector2(sfloat x, sfloat y) { this.x = x; this.y = y; }

        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public sVector2(float x, float y) { this.x = (sfloat)x; this.y = (sfloat)y; }

        // Set x and y components of an existing sVector2.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public void Set(sfloat newX, sfloat newY) { x = newX; y = newY; }

        // Linearly interpolates between two vectors.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sVector2 Lerp(sVector2 a, sVector2 b, sfloat t)
        {
            t = libm.Clamp01(t);
            return new sVector2(
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t
            );
        }

        // Linearly interpolates between two vectors without clamping the interpolant
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sVector2 LerpUnclamped(sVector2 a, sVector2 b, sfloat t)
        {
            return new sVector2(
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t
            );
        }

        // Moves a point /current/ towards /target/.
        public static sVector2 MoveTowards(sVector2 current, sVector2 target, sfloat maxDistanceDelta)
        {
            // avoid vector ops because current scripting backends are terrible at inlining
            sfloat toVector_x = target.x - current.x;
            sfloat toVector_y = target.y - current.y;

            sfloat sqDist = toVector_x * toVector_x + toVector_y * toVector_y;

            if (sqDist == sfloat.Zero || (maxDistanceDelta >= sfloat.Zero && sqDist <= maxDistanceDelta * maxDistanceDelta))
                return target;

            sfloat dist = (sfloat)libm.sqrtf(sqDist);

            return new sVector2(current.x + toVector_x / dist * maxDistanceDelta,
                current.y + toVector_y / dist * maxDistanceDelta);
        }

        // Multiplies two vectors component-wise.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sVector2 Scale(sVector2 a, sVector2 b) { return new sVector2(a.x * b.x, a.y * b.y); }

        // Multiplies every component of this vector by the same component of /scale/.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public void Scale(sVector2 scale) { x *= scale.x; y *= scale.y; }

        // Makes this vector have a ::ref::magnitude of 1.
        public void Normalize()
        {
            sfloat mag = magnitude;
            if (mag > kEpsilon)
                this = this / mag;
            else
                this = zero;
        }

        // Returns this vector with a ::ref::magnitude of 1 (RO).
        public sVector2 normalized
        {
            get
            {
                sVector2 v = new sVector2(x, y);
                v.Normalize();
                return v;
            }
        }

        /// *listonly*
        public override string ToString()
        {
            return ToString(null, CultureInfo.InvariantCulture.NumberFormat);
        }

        // Returns a nicely formatted string for this vector.
        public string ToString(string format)
        {
            return ToString(format, CultureInfo.InvariantCulture.NumberFormat);
        }

        public string ToString(string format, IFormatProvider formatProvider)
        {
            if (string.IsNullOrEmpty(format))
                format = "F1";
            return string.Format("({0}, {1})", x.ToString(format, formatProvider), y.ToString(format, formatProvider));
        }

        // used to allow sVector2s to be used as keys in hash tables
        public override int GetHashCode()
        {
            return x.GetHashCode() ^ (y.GetHashCode() << 2);
        }

        // also required for being able to use sVector2s as keys in hash tables
        public override bool Equals(object other)
        {
            if (!(other is sVector2)) return false;

            return Equals((sVector2)other);
        }

        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public bool Equals(sVector2 other)
        {
            return x == other.x && y == other.y;
        }

        public static sVector2 Reflect(sVector2 inDirection, sVector2 inNormal)
        {
            sfloat minusTwo = -(sfloat)2;
            sfloat factor = minusTwo * Dot(inNormal, inDirection);
            return new sVector2(factor * inNormal.x + inDirection.x, factor * inNormal.y + inDirection.y);
        }

        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sVector2 Perpendicular(sVector2 inDirection)
        {
            return new sVector2(-inDirection.y, inDirection.x);
        }

        // Dot Product of two vectors.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sfloat Dot(sVector2 lhs, sVector2 rhs) { return lhs.x * rhs.x + lhs.y * rhs.y; }

        // Returns the length of this vector (RO).
        public sfloat magnitude { get { return (sfloat)libm.sqrtf(x * x + y * y); } }
        // Returns the squared length of this vector (RO).
        public sfloat sqrMagnitude { get { return x * x + y * y; } }

        // Returns the angle in degrees between /from/ and /to/.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sfloat Angle(sVector2 from, sVector2 to)
        {
            // sqrt(a) * sqrt(b) = sqrt(a * b) -- valid for real numbers
            sfloat denominator = (sfloat)libm.sqrtf(from.sqrMagnitude * to.sqrMagnitude);
            if (denominator < kEpsilonNormalSqrt)
                return sfloat.Zero;

            sfloat dot = libm.Clamp(Dot(from, to) / denominator, sfloat.MinusOne, sfloat.One);
            return (sfloat)libm.acosf(dot) * libm.Rad2Deg;
        }

        // Returns the signed angle in degrees between /from/ and /to/. Always returns the smallest possible angle
        public static sfloat SignedAngle(sVector2 from, sVector2 to)
        {
            sfloat unsigned_angle = Angle(from, to);
            sfloat sign = libm.Sign(from.x * to.y - from.y * to.x);
            return unsigned_angle * sign;
        }

        // Returns the distance between /a/ and /b/.
        public static sfloat Distance(sVector2 a, sVector2 b)
        {
            sfloat diff_x = a.x - b.x;
            sfloat diff_y = a.y - b.y;
            return (sfloat)libm.sqrtf(diff_x * diff_x + diff_y * diff_y);
        }

        // Returns a copy of /vector/ with its magnitude clamped to /maxLength/.
        public static sVector2 ClampMagnitude(sVector2 vector, sfloat maxLength)
        {
            sfloat sqrMagnitude = vector.sqrMagnitude;
            if (sqrMagnitude > maxLength * maxLength)
            {
                sfloat mag = (sfloat)libm.sqrtf(sqrMagnitude);

                //these intermediate variables force the intermediate result to be
                //of sfloat precision. without this, the intermediate result can be of higher
                //precision, which changes behavior.
                sfloat normalized_x = vector.x / mag;
                sfloat normalized_y = vector.y / mag;
                return new sVector2(normalized_x * maxLength,
                    normalized_y * maxLength);
            }
            return vector;
        }

        // [Obsolete("Use sVector2.sqrMagnitude")]
        public static sfloat SqrMagnitude(sVector2 a) { return a.x * a.x + a.y * a.y; }
        // [Obsolete("Use sVector2.sqrMagnitude")]
        public sfloat SqrMagnitude() { return x * x + y * y; }

        // Returns a vector that is made from the smallest components of two vectors.
        public static sVector2 Min(sVector2 lhs, sVector2 rhs) { return new sVector2(sfloat.Min(lhs.x, rhs.x), sfloat.Min(lhs.y, rhs.y)); }

        // Returns a vector that is made from the largest components of two vectors.
        public static sVector2 Max(sVector2 lhs, sVector2 rhs) { return new sVector2(sfloat.Max(lhs.x, rhs.x), sfloat.Max(lhs.y, rhs.y)); }

        // [uei.ExcludeFromDocs]
        // public static sVector2 SmoothDamp(sVector2 current, sVector2 target, ref sVector2 currentVelocity, sfloat smoothTime, sfloat maxSpeed)
        // {
        //     sfloat deltaTime = Time.deltaTime;
        //     return SmoothDamp(current, target, ref currentVelocity, smoothTime, maxSpeed, deltaTime);
        // }

        // [uei.ExcludeFromDocs]
        // public static sVector2 SmoothDamp(sVector2 current, sVector2 target, ref sVector2 currentVelocity, sfloat smoothTime)
        // {
        //     sfloat deltaTime = Time.deltaTime;
        //     sfloat maxSpeed = Mathf.Infinity;
        //     return SmoothDamp(current, target, ref currentVelocity, smoothTime, maxSpeed, deltaTime);
        // }

        // public static sVector2 SmoothDamp(sVector2 current, sVector2 target, ref sVector2 currentVelocity, sfloat smoothTime, [uei.DefaultValue("Mathf.Infinity")] sfloat maxSpeed, [uei.DefaultValue("Time.deltaTime")] sfloat deltaTime)
        // {
        //     // Based on Game Programming Gems 4 Chapter 1.10
        //     smoothTime = Mathf.Max(0.0001F, smoothTime);
        //     sfloat omega = 2F / smoothTime;

        //     sfloat x = omega * deltaTime;
        //     sfloat exp = 1F / (1F + x + 0.48F * x * x + 0.235F * x * x * x);

        //     sfloat change_x = current.x - target.x;
        //     sfloat change_y = current.y - target.y;
        //     sVector2 originalTo = target;

        //     // Clamp maximum speed
        //     sfloat maxChange = maxSpeed * smoothTime;

        //     sfloat maxChangeSq = maxChange * maxChange;
        //     sfloat sqDist = change_x * change_x + change_y * change_y;
        //     if (sqDist > maxChangeSq)
        //     {
        //         var mag = (sfloat)libm.sqrtf(sqDist);
        //         change_x = change_x / mag * maxChange;
        //         change_y = change_y / mag * maxChange;
        //     }

        //     target.x = current.x - change_x;
        //     target.y = current.y - change_y;

        //     sfloat temp_x = (currentVelocity.x + omega * change_x) * deltaTime;
        //     sfloat temp_y = (currentVelocity.y + omega * change_y) * deltaTime;

        //     currentVelocity.x = (currentVelocity.x - omega * temp_x) * exp;
        //     currentVelocity.y = (currentVelocity.y - omega * temp_y) * exp;

        //     sfloat output_x = target.x + (change_x + temp_x) * exp;
        //     sfloat output_y = target.y + (change_y + temp_y) * exp;

        //     // Prevent overshooting
        //     sfloat origMinusCurrent_x = originalTo.x - current.x;
        //     sfloat origMinusCurrent_y = originalTo.y - current.y;
        //     sfloat outMinusOrig_x = output_x - originalTo.x;
        //     sfloat outMinusOrig_y = output_y - originalTo.y;

        //     if (origMinusCurrent_x * outMinusOrig_x + origMinusCurrent_y * outMinusOrig_y > 0)
        //     {
        //         output_x = originalTo.x;
        //         output_y = originalTo.y;

        //         currentVelocity.x = (output_x - originalTo.x) / deltaTime;
        //         currentVelocity.y = (output_y - originalTo.y) / deltaTime;
        //     }
        //     return new sVector2(output_x, output_y);
        // }

        // Adds two vectors.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sVector2 operator+(sVector2 a, sVector2 b) { return new sVector2(a.x + b.x, a.y + b.y); }
        // Subtracts one vector from another.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sVector2 operator-(sVector2 a, sVector2 b) { return new sVector2(a.x - b.x, a.y - b.y); }
        // Multiplies one vector by another.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sVector2 operator*(sVector2 a, sVector2 b) { return new sVector2(a.x * b.x, a.y * b.y); }
        // Divides one vector over another.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sVector2 operator/(sVector2 a, sVector2 b) { return new sVector2(a.x / b.x, a.y / b.y); }
        // Negates a vector.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sVector2 operator-(sVector2 a) { return new sVector2(-a.x, -a.y); }
        // Multiplies a vector by a number.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sVector2 operator*(sVector2 a, sfloat d) { return new sVector2(a.x * d, a.y * d); }
        // Multiplies a vector by a number.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sVector2 operator*(sfloat d, sVector2 a) { return new sVector2(a.x * d, a.y * d); }
        // Divides a vector by a number.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static sVector2 operator/(sVector2 a, sfloat d) { return new sVector2(a.x / d, a.y / d); }
        // Returns true if the vectors are equal.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static bool operator==(sVector2 lhs, sVector2 rhs)
        {
            // Returns false in the presence of NaN values.
            sfloat diff_x = lhs.x - rhs.x;
            sfloat diff_y = lhs.y - rhs.y;
            return (diff_x * diff_x + diff_y * diff_y) < kEpsilon * kEpsilon;
        }

        // Returns true if vectors are different.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static bool operator!=(sVector2 lhs, sVector2 rhs)
        {
            // Returns true in the presence of NaN values.
            return !(lhs == rhs);
        }

        // // Converts a [[Vector3]] to a sVector2.
        // [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        // public static implicit operator sVector2(Vector3 v)
        // {
        //     return new sVector2(v.x, v.y);
        // }

        // // Converts a sVector2 to a [[Vector3]].
        // [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        // public static implicit operator Vector3(sVector2 v)
        // {
        //     return new Vector3(v.x, v.y, 0);
        // }

        static readonly sVector2 zeroVector = new sVector2(0F, 0F);
        static readonly sVector2 oneVector = new sVector2(1F, 1F);
        static readonly sVector2 upVector = new sVector2(0F, 1F);
        static readonly sVector2 downVector = new sVector2(0F, -1F);
        static readonly sVector2 leftVector = new sVector2(-1F, 0F);
        static readonly sVector2 rightVector = new sVector2(1F, 0F);
        static readonly sVector2 positiveInfinityVector = new sVector2(sfloat.PositiveInfinity, sfloat.PositiveInfinity);
        static readonly sVector2 negativeInfinityVector = new sVector2(sfloat.NegativeInfinity, sfloat.NegativeInfinity);


        // Shorthand for writing @@sVector2(0, 0)@@
        public static sVector2 zero { get { return zeroVector; } }
        // Shorthand for writing @@sVector2(1, 1)@@
        public static sVector2 one { get { return oneVector; }   }
        // Shorthand for writing @@sVector2(0, 1)@@
        public static sVector2 up { get { return upVector; } }
        // Shorthand for writing @@sVector2(0, -1)@@
        public static sVector2 down { get { return downVector; } }
        // Shorthand for writing @@sVector2(-1, 0)@@
        public static sVector2 left { get { return leftVector; } }
        // Shorthand for writing @@sVector2(1, 0)@@
        public static sVector2 right { get { return rightVector; } }
        // Shorthand for writing @@sVector2(sfloat.PositiveInfinity, sfloat.PositiveInfinity)@@
        public static sVector2 positiveInfinity { get { return positiveInfinityVector; } }
        // Shorthand for writing @@sVector2(sfloat.NegativeInfinity, sfloat.NegativeInfinity)@@
        public static sVector2 negativeInfinity { get { return negativeInfinityVector; } }

        // *Undocumented*
        public static sfloat kEpsilon = (sfloat)0.00001F;
        // *Undocumented*
        public static sfloat kEpsilonNormalSqrt = (sfloat)1e-15f;
    }
}