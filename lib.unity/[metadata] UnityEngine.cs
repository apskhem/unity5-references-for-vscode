using System;
using System.Collections;
using System.Collections.Generic;

/// <summary>
/// Use ForceMode to specify how to apply a force using <c>Rigidbody.AddForce</c>.
/// </summary>
enum ForceMode {
    /// <summary>
    /// Add a continuous force to the rigidbody, using its mass.
    /// </summary>
    Force,
    /// <summary>
    /// Add a continuous acceleration to the rigidbody, ignoring its mass.
    /// </summary>
    Acceleration,
    /// <summary>
    /// Add an instant force impulse to the rigidbody, using its mass.
    /// </summary>
    Impulse,
    /// <summary>
    /// Add an instant velocity change to the rigidbody, ignoring its mass.
    /// </summary>
    VelocityChange
}

/// <summary>
/// The various primitives that can be created using the GameObject.CreatePrimitive function.
/// </summary>
enum PrimitiveType {
    /// <summary>
    /// A sphere primitive.
    /// </summary>
    Sphere,
    /// <summary>
    /// A capsule primitive.
    /// </summary>
    Capsule,
    /// <summary>
    /// A cylinder primitive.
    /// </summary>
    Cylinder,
    /// <summary>
    /// A cube primitive.
    /// </summary>
    Cube,
    /// <summary>
    /// A plane primitive.
    /// </summary>
    Plane,
    /// <summary>
    /// A quad primitive.
    /// </summary>
    Quad
}

enum Space {
    /// <summary>
    /// Applies transformation relative to the world coordinate system.
    /// </summary>
    World,
    /// <summary>
    /// Applies transformation relative to the local coordinate system.
    /// </summary>
    Self
}

namespace UnityEngine {

    /// <summary>
    /// Structure describing acceleration status of the device.
    /// </summary>
    public struct AccelerationEvent {
        /// <summary>
        /// Value of acceleration.
        /// </summary>
        public Vector3 acceleration;
        /// <summary>
        /// Amount of time passed since last accelerometer measurement.
        /// </summary>
        public float deltaTime;
    }

    /// <summary>
    /// Representation of RGBA colors.
    /// <para>
    /// This structure is used throughout Unity to pass colors around. Each color component is a floating point value with a range from 0 to 1.
    /// </para>
    /// Components (r,g,b) define a color in RGB color space.
    /// Alpha component (a) defines transparency - alpha of one is completely opaque, alpha of zero is completely transparent.
    /// </summary>
    public struct Color {
        /// <summary>
        /// Solid black. RGBA is (0, 0, 0, 1).
        /// </summary>
        public static Color black;
        /// <summary>
        /// Solid blue. RGBA is (0, 0, 1, 1).
        /// </summary>
        public static Color blue;
        /// <summary>
        /// Completely transparent. RGBA is (0, 0, 0, 0).
        /// </summary>
        public static Color clear;
        /// <summary>
        /// Cyan. RGBA is (0, 1, 1, 1).
        /// </summary>
        public static Color cyan;
        /// <summary>
        /// Gray. RGBA is (0.5, 0.5, 0.5, 1).
        /// </summary>
        public static Color gray;
        /// <summary>
        /// Solid green. RGBA is (0, 1, 0, 1).
        /// </summary>
        public static Color green;
        /// <summary>
        /// English spelling for gray. RGBA is the same (0.5, 0.5, 0.5, 1).
        /// </summary>
        public static Color grey;
        /// <summary>
        /// Magenta. RGBA is (1, 0, 1, 1).
        /// </summary>
        public static Color magenta;
        /// <summary>
        /// Solid red. RGBA is (1, 0, 0, 1).
        /// </summary>
        public static Color red;
        /// <summary>
        /// Solid white. RGBA is (1, 1, 1, 1).
        /// </summary>
        public static Color white;
        /// <summary>
        /// Yellow. RGBA is (1, 0.92, 0.016, 1), but the color is nice to look at!
        /// </summary>
        public static Color yellow;


        /// <summary>
        /// Alpha component of the color (0 is transparent, 1 is opaque).
        /// </summary>
        public float a;
        /// <summary>
        /// Blue component of the color.
        /// </summary>
        public float b;
        /// <summary>
        /// Green component of the color.
        /// </summary>
        public float g;
        /// <summary>
        /// A version of the color that has had the gamma curve applied.
        /// </summary>
        public Color gamma;
        /// <summary>
        /// The grayscale value of the color. (Read Only)
        /// </summary>
        public readonly float grayscale;
        /// <summary>
        /// A linear value of an sRGB color.
        /// </summary>
        public Color linear;
        /// <summary>
        /// Returns the maximum color component value: Max(r,g,b).
        /// </summary>
        public float maxColorComponent;
        /// <summary>
        /// Red component of the color.
        /// </summary>
        public float r;


        /// <summary>
        /// Access the r, g, b, a components using [0], [1], [2], [3] respectively.
        /// </summary>
        public float this[int index] {}


        /// <summary>
        /// Constructs a new Color with given r,g,b,a components.
        /// </summary>
        /// <param name="r">Red component.</param>
        /// <param name="g">Green component.</param>
        /// <param name="b">Blue component.</param>
        /// <param name="a">Alpha component.</param>
        public Color(float r, float g, float b, float a);


        /// <summary>
        /// Returns a nicely formatted string of this color.
        /// </summary>
        public string ToString();
        public string ToString(string format);

        
        /// <summary>
        /// Creates an RGB colour from HSV input.
        /// </summary>
        /// <param name="H">Hue [0..1].</param>
        /// <param name="S">Saturation [0..1].</param>
        /// <param name="V">Brightness value [0..1].</param>
        /// <param name="hdr">Output HDR colours. If true, the returned colour will not be clamped to [0..1].</param>
        /// <returns>Color An opaque colour with HSV matching the input.</returns>
        public static Color HSVToRGB(float H, float S, float V);
        public static Color HSVToRGB(float H, float S, float V, bool hdr);
        /// <summary>
        /// Linearly interpolates between colors <c>a</c> and <c>b</c> by <c>t</c>.
        /// </summary>
        /// <param name="a">Color a.</param>
        /// <param name="b">Color b.</param>
        /// <param name="t">Float for combining a and b.</param>
        public static Color Lerp(Color a, Color b, float t);
        /// <summary>
        /// Linearly interpolates between colors <c>a</c> and <c>b</c> by <c>t</c>.
        /// </summary>
        /// </summary>
        /// <param name="a">Color a.</param>
        /// <param name="b">Color b.</param>
        /// <param name="t">Float for combining a and b.</param>
        public static Color LerpUnclamped(Color a, Color b, float t);
        /// <summary>
        /// Calculates the hue, saturation and value of an RGB input color.
        /// </summary>
        /// <param name="rgbColor">An input color.</param>
        /// <param name="H">Output variable for hue.</param>
        /// <param name="S">Output variable for saturation.</param>
        /// <param name="V">Output variable for value.</param>
        public static void RGBToHSV(Color rgbColor, out float H, out float S, out float V);


        /// <summary>
        /// Colors can be implicitly converted to and from Vector4.
        /// </summary>
        public static implicit operator Color(Vector4 v);
        /// <summary>
        /// Subtracts color b from color a. Each component is subtracted separately.
        /// </summary>
        public static Color operator -(Color a, float d);
        /// <summary>
        /// Multiplies two colors together. Each component is multiplied separately.
        /// </summary>
        public static Color operator *(Color a, float d);
        /// <summary>
        /// Divides color a by the float b. Each color component is scaled separately.
        /// </summary>
        public static Color operator /(Color a, float d);
        /// <summary>
        /// Adds two colors together. Each component is added separately.
        /// </summary>
        public static Color operator +(Color a, float d);
        /// <summary>
        /// Colors can be implicitly converted to and from Vector4.
        /// </summary>
        public static implicit operator Vector4(Color c);
    }

    /// <summary>
    /// Quaternions are used to represent rotations.
    /// <para>
    /// They are compact, don't suffer from gimbal lock and can easily be interpolated.
    /// Unity internally uses Quaternions to represent all rotations.
    /// </para>
    /// They are based on complex numbers and are not easy to understand intuitively.
    /// You almost never access or modify individual Quaternion components (x,y,z,w);
    /// most often you would just take existing rotations (e.g. from the Transform) and use them to construct new rotations (e.g. to smoothly interpolate between two rotations).
    /// The Quaternion functions that you use 99% of the time are: Quaternion.LookRotation, Quaternion.Angle, Quaternion.Euler, Quaternion.Slerp, Quaternion.FromToRotation, and Quaternion.identity.
    /// (The other functions are only for exotic uses.)
    /// </summary>
    public struct Quaternion {
        /// <summary>
        /// The identity rotation (Read Only).
        /// </summary>
        public static readonly Quaternion identity;


        /// <summary>
        /// Returns or sets the euler angle representation of the rotation.
        /// </summary>
        public Vector3 eulerAngles;
        /// <summary>
        /// Returns this quaternion with a magnitude of 1 (Read Only).
        /// </summary>
        public readonly Quaternion normalized;
        /// <summary>
        /// Access the x, y, z, w components using [0], [1], [2], [3] respectively.
        /// </summary>
        public float this[int index] {}
        /// <summary>
        /// W component of the Quaternion. Do not directly modify quaternions.
        /// </summary>
        public float w;
        /// <summary>
        /// X component of the Quaternion. Don't modify this directly unless you know quaternions inside out.
        /// </summary>
        public float x;
        /// <summary>
        /// Y component of the Quaternion. Don't modify this directly unless you know quaternions inside out.
        /// </summary>
        public float y;
        /// <summary>
        /// Z component of the Quaternion. Don't modify this directly unless you know quaternions inside out.
        /// </summary>
        public float z;


        /// <summary>
        /// Constructs new Quaternion with given x,y,z,w components.
        /// </summary>
        /// <param name="x">X component of the Quaternion.</param>
        /// <param name="y">Y component of the Quaternion.</param>
        /// <param name="z">Z component of the Quaternion.</param>
        /// <param name="w">W component of the Quaternion.</param>
        public Quaternion(float x, float y, float z, float w);
    
    
        /// <summary>
        /// Set x, y, z and w components of an existing Quaternion.
        /// </summary>
        public void Set(float newX, float newY, float newZ, float newW);
        /// <summary>
        /// Creates a rotation which rotates from <c>fromDirection</c> to <c>toDirection</c>.
        /// </summary>
        public void SetFromToRotation(Vector3 fromDirection, Vector3 toDirection);
        /// <summary>
        /// Creates a rotation with the specified <c>forward</c> and <c>upwards</c> directions.
        /// </summary>
        /// <param name="view">The direction to look in.</param>
        /// <param name="up">The vector that defines in which direction up is.</param>
        public void SetLookRotation(Vector3 view, Vector3 up = Vector3.up);
        /// <summary>
        /// Converts a rotation to angle-axis representation (angles in degrees).
        /// </summary>
        public void ToAngleAxis(out float angle, out Vector3 axis);
        /// <summary>
        /// Returns a nicely formatted string of the Quaternion.
        /// </summary>
        public string ToString();
        public string ToString(string format);


        /// <summary>
        /// Returns the angle in degrees between two rotations <c>a</c> and <c>b</c>.
        /// </summary>
        public static float Angle(Quaternion a, Quaternion b);
        /// <summary>
        /// Creates a rotation which rotates <c>angle</c> degrees around <c>axis</c>.
        /// </summary>
        public static Quaternion AngleAxis(float angle, Vector3 axis);
        /// <summary>
        /// The dot product between two rotations.
        /// </summary>
        public static float Dot(Quaternion a, Quaternion b);
        /// <summary>
        /// Returns a rotation that rotates z degrees around the z axis, x degrees around the x axis, and y degrees around the y axis; applied in that order.
        /// </summary>
        public static Quaternion Euler(float x, float y, float z);
        /// <summary>
        /// Creates a rotation which rotates from <c>fromDirection</c> to <c>toDirection</c>.
        /// </summary>
        public static Quaternion FromToRotation(Vector3 fromDirection, Vector3 toDirection);
        /// <summary>
        /// Returns the Inverse of <c>rotation</c>.
        /// </summary>
        public static Quaternion Inverse(Quaternion rotation);
        /// <summary>
        /// Interpolates between <c>a</c> and <c>b</c> by <c>t</c> and normalizes the result afterwards. The parameter <c>t</c> is clamped to the range [0, 1].
        /// </summary>
        public static Quaternion Lerp(Quaternion a, Quaternion b, float t);
        /// <summary>
        /// Interpolates between <c>a</c> and <c>b</c> by <c>t</c> and normalizes the result afterwards. The parameter <c>t</c> is not clamped.
        /// </summary>
        public static Quaternion LerpUnclamped(Quaternion a, Quaternion b, float t);
        /// <summary>
        /// Creates a rotation with the specified <c>forward</c> and <c>upwards</c> directions.
        /// </summary>
        /// <param name="forward">The direction to look in.</param>
        /// <param name="upwards">The vector that defines in which direction up is.</param>
        public static Quaternion LookRotation(Vector3 forward, Vector3 upwards = Vector3.up);
        /// <summary>
        /// Converts this quaternion to one with the same orientation but with a magnitude of 1.
        /// </summary>
        public static Quaternion Normalize(Quaternion q);
        /// <summary>
        /// Rotates a rotation <c>from</c> towards <c>to</c>.
        /// </summary>
        public static Quaternion RotateTowards(Quaternion from, Quaternion to, float maxDegreesDelta);
        /// <summary>
        /// Spherically interpolates between quaternions <c>a</c> and <c>b</c> by ratio <c>t</c>. The parameter <c>t</c> is clamped to the range [0, 1].
        /// </summary>
        /// <param name="a">Start value, returned when <c>t</c> = 0.</param>
        /// <param name="b">End value, returned when <c>t</c> = 1.</param>
        /// <param name="t">Interpolation ratio.</param>
        /// <returns>Quaternion A quaternion spherically interpolated between quaternions <c>a</c> and <c>b</c>.</returns>
        public static Quaternion Slerp(Quaternion a, Quaternion b, float t);
        /// <summary>
        /// Spherically interpolates between <c>a</c> and <c>b</c> by <c>t</c>. The parameter <c>t</c> is not clamped.
        /// </summary>
        /// <param name="a">Start value, returned when <c>t</c> = 0.</param>
        /// <param name="b">End value, returned when <c>t</c> = 1.</param>
        /// <param name="t">Interpolation ratio.</param>
        public static Quaternion SlerpUnclamped(Quaternion a, Quaternion b, float t);


        /// <summary>
        /// Combines rotations lhs and rhs.
        /// </summary>
        public static Vector3 operator *(Vector3 a, float d);
        /// <summary>
        /// Are two quaternions equal to each other?
        /// </summary>
        public static Vector3 operator ==(Vector3 a, float d);
    }

    /// <summary>
    /// Representation of 3D vectors and points.
    /// <para>
    /// This structure is used throughout Unity to pass 3D positions and directions around.
    /// It also contains functions for doing common vector operations.
    /// </para>
    /// Besides the functions listed below, other classes can be used to manipulate vectors and points as well.
    /// For example the Quaternion and the Matrix4x4 classes are useful for rotating or transforming vectors and points.
    /// </summary>
    public struct Vector3 {
        /// <summary>
        /// Shorthand for writing <c>Vector3(0, 0, -1)</c>.
        /// </summary>
        public static Vector3 back;
        /// <summary>
        /// Shorthand for writing <c>Vector3(0, -1, 0)</c>.
        /// </summary>
        public static Vector3 down;
        /// <summary>
        /// Shorthand for writing <c>Vector3(0, 0, 11)</c>.
        /// </summary>
        public static Vector3 forward;
        /// <summary>
        /// Shorthand for writing <c>Vector3(-1, 0, 0)</c>.
        /// </summary>
        public static Vector3 left;
        /// <summary>
        /// Shorthand for writing <c>Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity)</c>.
        /// </summary>
        public static Vector3 negativeInfinity;
        /// <summary>
        /// Shorthand for writing <c>Vector3(1, 1, 1)</c>.
        /// </summary>
        public static Vector3 one;
        /// <summary>
        /// Shorthand for writing <c>Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity)</c>.
        /// </summary>
        public static Vector3 positiveInfinity;
        /// <summary>
        /// Shorthand for writing <c>Vector3(1, 0, 0)</c>.
        /// </summary>
        public static Vector3 right;
        /// <summary>
        /// Shorthand for writing <c>Vector3(0, 1, 0)</c>.
        /// </summary>
        public static Vector3 up;
        /// <summary>
        /// Shorthand for writing <c>Vector3(0, 0, 0)</c>.
        /// </summary>
        public static Vector3 zero;


        /// <summary>
        /// Returns the length of this vector (Read Only).
        /// </summary>
        public readonly float magnitude;
        /// <summary>
        /// Returns this vector with a magnitude of 1 (Read Only).
        /// </summary>
        public readonly Vector3 normalized;
        /// <summary>
        /// Returns the squared length of this vector (Read Only).
        /// </summary>
        public readonly float sqrMagnitude;
        /// <summary>
        /// Access the x, y, z components using [0], [1], [2] respectively.
        /// </summary>
        public float this[int index] {}
        /// <summary>
        /// X component of the vector.
        /// </summary>
        public float x;
        /// <summary>
        /// Y component of the vector.
        /// </summary>
        public float y;
        /// <summary>
        /// Z component of the vector.
        /// </summary>
        public float z;

        /// <summary>
        /// Creates a new vector with given x, y, z components.
        /// </summary>
        /// <param name="x">X component of the vector.</param>
        /// <param name="y">Y component of the vector.</param>
        /// <param name="z">Z component of the vector.</param>
        public Vector3(float x, float y, float z);


        /// <summary>
        /// Returns true if the given <c>vector</c> is exactly equal to this <c>vector</c>.
        /// </summary>
        /// <param name="other">Other vector to be compared.</param>
        public bool Equals(object other);
        /// <summary>
        /// Set x, y and z components of an existing Vector3.
        /// </summary>
        /// <param name="newX">The new x component to be set.</param>
        /// <param name="newY">The new y component to be set.</param>
        /// <param name="newZ">The new z component to be set.</param>
        public void Set(float newX, float newY, float newZ);
        /// <summary>
        /// Returns a nicely formatted string for this vector.
        /// </summary>
        public string ToString();
        public string ToString(string format);

        /// <summary>
        /// Returns the angle in degrees between <c>from</c> and <c>to</c>.
        /// </summary>
        /// <param name="from">The vector from which the angular difference is measured.</param>
        /// <param name="to">The vector to which the angular difference is measured.</param>
        /// <returns>float The angle in degrees between the two vectors.</returns>
        public static float Angle(Vector3 from, Vector3 to);
        /// <summary>
        /// Returns a copy of <c>vector</c> with its magnitude clamped to <c>maxLength</c>.
        /// </summary>
        /// <param name="vector"></param>
        /// <param name="maxLength"></param>
        /// <returns></returns>
        public static Vector3 ClampMagnitude(Vector3 vector, float maxLength);
        /// <summary>
        /// Cross Product of two vectors.
        /// </summary>
        /// <param name="lhs">Left hand rule.</param>
        /// <param name="rhs">Right hand rule.</param>
        /// <returns>Third vector which is perpendicular to the two input vectors</returns>
        public static Vector3 Cross(Vector3 lhs, Vector3 rhs);
        /// <summary>
        /// Returns the distance between <c>a</c> and <c>b</c>.
        /// </summary>
        public static float Distance(Vector3 a, Vector3 b);
        /// <summary>
        /// Dot Product of two vectors.
        /// </summary>
        /// <param name="lhs">Left hand rule.</param>
        /// <param name="rhs">Right hand rule.</param>
        public static float Dot(Vector3 lhs, Vector3 rhs);
        /// <summary>
        /// Linearly interpolates between two points.
        /// </summary>
        /// <param name="a">Start value, returned when <c>t</c> = 0.</param>
        /// <param name="b">End value, returned when <c>t</c> = 1.</param>
        /// <param name="t">Value used to interpolate between <c>a</c> and <c>b</c>.</param>
        /// <returns>Vector3 Interpolated value, equals to <c>a</c> + (<c>b</c> - <c>a</c>) * <c>t</c>.</returns>
        public static Vector3 Lerp(Vector3 a, Vector3 b, float t);
        /// <summary>
        /// Linearly interpolates between two vectors.
        /// </summary>
        /// <param name="a">Start value, returned when <c>t</c> = 0.</param>
        /// <param name="b">End value, returned when <c>t</c> = 1.</param>
        /// <param name="t">Value used to interpolate between <c>a</c> and <c>b</c>.</param>
        /// <returns>The point midway between <c>a</c> and <c>b</c>.</returns>
        public static Vector3 LerpUnclamped(Vector3 a, Vector3 b, float t);
        /// <summary>
        /// Returns a vector that is made from the largest components of two vectors.
        /// </summary>
        /// <param name="lhs">Left hand rule.</param>
        /// <param name="rhs">Right hand rule.</param>
        public static Vector3 Max(Vector3 lhs, Vector3 rhs);
        /// <summary>
        /// Returns a vector that is made from the smallest components of two vectors.
        /// </summary>
        /// <param name="lhs">Left hand rule.</param>
        /// <param name="rhs">Right hand rule.</param>
        public static Vector3 Min(Vector3 lhs, Vector3 rhs);
        /// <summary>
        /// Calculate a position between the points specified by <c>current</c> and <c>target</c>, moving no farther than the distance specified by <c>maxDistanceDelta</c>.
        /// </summary>
        /// <param name="current">The position to move from.</param>
        /// <param name="target">The position to move towards.</param>
        /// <param name="maxDistanceDelta">Distance to move <c>current</c> per call.</param>
        /// <returns><c>Vector3</c> The new position.</returns>
        public static Vector3 MoveTowards(Vector3 current, Vector3 target, float maxDistanceDelta);
        /// <summary>
        /// Makes this vector have a magnitude of 1.
        /// </summary>
        /// <param name="value">The vector to be normalized.</param>
        /// <returns>The normalized vector</returns>
        public static Vector3 Normalize(Vector3 value);
        /// <summary>
        /// Makes vectors normalized and orthogonal to each other.
        /// </summary>
        public static void OrthoNormalize(ref Vector3 normal, ref Vector3 tangent);
        /// <summary>
        /// Projects a vector onto another vector.
        /// </summary>
        public static Vector3 Project(Vector3 vector, Vector3 onNormal);
        /// <summary>
        /// Projects a vector onto a plane defined by a normal orthogonal to the plane.
        /// </summary>
        /// <param name="vector">The direction from the vector towards the plane.</param>
        /// <param name="planeNormal">The location of the vector above the plane.</param>
        /// <returns><c>Vector3</c> The location of the vector on the plane.</returns>
        public static Vector3 ProjectOnPlane(Vector3 vector, Vector3 planeNormal);
        /// <summary>
        /// Reflects a vector off the plane defined by a normal.
        /// </summary>
        public static Vector3 Reflect(Vector3 inDirection, Vector3 inNormal);
        /// <summary>
        /// Rotates a vector <c>current</c> towards <c>target</c>.
        /// </summary>
        /// <param name="current">The vector being managed.</param>
        /// <param name="target">The vector.</param>
        /// <param name="maxRadiansDelta">The maximum angle in radians allowed for this rotation.</param>
        /// <param name="maxMagnitudeDelta">The maximum allowed change in vector magnitude for this rotation.</param>
        /// <returns><c>Vector3</c> The location that RotateTowards generates.</returns>
        public static Vector3 RotateTowards(Vector3 current, Vector3 target, float maxRadiansDelta, float maxMagnitudeDelta);
        /// <summary>
        /// Multiplies two vectors component-wise.
        /// </summary>
        public static Vector3 Scale(Vector3 a, Vector3 b);
        /// <summary>
        /// Returns the signed angle in degrees between <c>from</c> and <c>to</c>.
        /// </summary>
        /// <param name="from">The vector from which the angular difference is measured.</param>
        /// <param name="to">The vector to which the angular difference is measured.</param>
        /// <param name="axis">A vector around which the other vectors are rotated.</param>
        public static float SignedAngle(Vector3 from, Vector3 to, Vector3 axis);
        /// <summary>
        /// Spherically interpolates between two vectors.
        /// </summary>
        public static Vector3 Slerp(Vector3 a, Vector3 b, float t);
        /// <summary>
        /// Spherically interpolates between two vectors.
        /// </summary>
        public static Vector3 SlerpUnclamped(Vector3 a, Vector3 b, float t);
        /// <summary>
        /// Gradually changes a vector towards a desired goal over time.
        /// </summary>
        /// <param name="current">The current position.</param>
        /// <param name="target">The position we are trying to reach.</param>
        /// <param name="currentVelocity">The current velocity, this value is modified by the function every time you call it.</param>
        /// <param name="smoothTime">Approximately the time it will take to reach the target. A smaller value will reach the target faster.</param>
        /// <param name="maxSpeed">Optionally allows you to clamp the maximum speed.</param>
        /// <param name="deltaTime">The time since the last call to this function. By default Time.deltaTime.</param>
        public static Vector3 SmoothDamp(Vector3 current, Vector3 target, ref Vector3 currentVelocity, float smoothTime, float maxSpeed = Mathf.Infinity, float deltaTime = Time.deltaTime);
    

        /// <summary>
        /// Subtracts one vector from another.
        /// </summary>
        public static Vector3 operator -(Vector3 a, Vector3 b);
        /// <summary>
        /// Returns true if vectors different.
        /// </summary>
        public static bool operator !=(Vector3 lhs, Vector3 rhs);
        /// <summary>
        /// Multiplies a vector by a number.
        /// </summary>
        public static Vector3 operator *(Vector3 a, float d);
        /// <summary>
        /// Divides a vector by a number.
        /// </summary>
        public static Vector3 operator /(Vector3 a, float d);
        /// <summary>
        /// Adds two vectors.
        /// </summary>
        public static Vector3 operator +(Vector3 a, float d);
        /// <summary>
        /// Returns true if two vectors are approximately equal.
        /// </summary>
        public static Vector3 operator ==(Vector3 a, float d);

    }

    /// <summary>
    /// Behaviours are Components that can be enabled or disabled.
    /// </summary>
    public class Behaviour : Component {
        /// <summary>
        /// Enabled Behaviours are Updated, disabled Behaviours are not.
        /// </summary>
        public bool enabled;
        /// <summary>
        /// Has the Behaviour had active and enabled called?
        /// </summary>
        public bool isActiveAndEnabled;
    }

    /// <summary>
    /// A box-shaped primitive collider.
    /// </summary>
    public class BoxCollider : Collider {
        /// <summary>
        /// The center of the box, measured in the object's local space.
        /// </summary>
        public Vector3 center;
        /// <summary>
        /// The size of the box, measured in the object's local space.
        /// </summary>
        public Vector3 size;
    }

    /// <summary>
    /// A capsule-shaped primitive collider.
    /// </summary>
    public class CapsuleCollider : Collider {
        /// <summary>
        /// The center of the capsule, measured in the object's local space.
        /// </summary>
        public Vector3 center;
        /// <summary>
        /// The direction of the capsule.
        /// </summary>
        public int direction;
        /// <summary>
        /// The height of the capsule measured in the object's local space.
        /// </summary>
        public float height;
        /// <summary>
        /// The radius of the sphere, measured in the object's local space.
        /// </summary>
        public float radius;
    }

    /// <summary>
    /// Base class for everything attached to GameObjects.
    /// </summary>
    public class Component : Object {
        /// <summary>
        /// The game object this component is attached to. A component is always attached to a game object.
        /// </summary>
        public GameObject gameObject;
        /// <summary>
        /// The tag of this game object.
        /// </summary>
        public string tag;
        /// <summary>
        /// The Transform attached to this GameObject.
        /// </summary>
        public Transform transform;


        /// <summary>
        /// Calls the method named methodName on every MonoBehaviour in this game object or any of its children.
        /// </summary>
        /// <param name="methodName">Name of coroutine.</param>
        /// <param name="parameter">Optional parameter to pass to the method (can be any value).</param>
        /// <param name="options">Should an error be raised if the method does not exist for a given target object?</param>
        public void BroadcastMessage(string methodName, object parameter = null, SendMessageOptions options = SendMessageOptions.RequireReceiver);
        public void BroadcastMessage(string methodName, SendMessageOptions options);
        /// <summary>
        /// Is this game object tagged with <c>tag</c>?
        /// </summary>
        /// <param name="tag">The tag to compare.</param>
        public bool CompareTag(string tag);
        /// <summary>
        /// Returns the component of Type <c>type</c> if the game object has one attached, null if it doesn't.
        /// </summary>
        public T GetComponent<T>() where T : Component;
        /// <summary>
        /// Returns the component of Type <c>type</c> in the GameObject or any of its children using depth first search.
        /// </summary>
        /// <returns><c>Component</c> A component of the matching type, if found.</returns>
        public T GetComponentInChildren<T>() where T : Component;
        /// <summary>
        /// Returns the component of Type <c>type</c> in the <c>GameObject</c> or any of its parents.
        /// </summary>
        public T GetComponentInParent<T>() where T : Component;
        /// <summary>
        /// Returns all components of Type <c>type</c> in the <c>GameObject</c>.
        /// </summary>
        public T[] GetComponents<T>() where T : Component;
        /// <summary>
        /// Returns all components of Type type in the <c>GameObject</c> or any of its children.
        /// </summary>
        public T[] GetComponentsInChildren<T>() where T : Component;
        /// <summary>
        /// Returns all components of Type type in the <c>GameObject</c> or any of its parents.
        /// </summary>
        public T[] GetComponentsInParent<T>() where T : Component;
        /// <summary>
        /// Calls the method named methodName on every <c>MonoBehaviour</c> in this game object.
        /// </summary>
        /// <param name="methodName">Name of the method to call.</param>
        /// <param name="value">Optional parameter for the method.</param>
        /// <param name="options">Should an error be raised if the target object doesn't implement the method for the message?</param>
        public void SendMessage(string methodName);
        public void SendMessage(string methodName, object value);
        public void SendMessage(string methodName, object value, SendMessageOptions options);
        public void SendMessage(string methodName, SendMessageOptions options);
        /// <summary>
        /// Calls the method named methodName on every MonoBehaviour in this game object and on every ancestor of the behaviour.
        /// </summary>
        /// <param name="methodName">Name of method to call.</param>
        /// <param name="value">Optional parameter value for the method.</param>
        /// <param name="options">Should an error be raised if the method does not exist on the target object?</param>
        public void SendMessageUpwards(string methodName, SendMessageOptions options);
        public void SendMessageUpwards(string methodName, object value = null, SendMessageOptions options = SendMessageOptions.RequireReceiver);
        /// <summary>
        /// Gets the component of the specified type, if it exists.
        /// </summary>
        /// <param name="type">The type of the component to retrieve.</param>
        /// <param name="component">The output argument that will contain the component or <c>null</c>.</param>
        /// <returns><c>bool</c> Returns <c>true</c> if the component is found, <c>false</c> otherwise.</returns>
        public bool TryGetComponent(Type type, out Component component);
        public bool TryGetComponent(out T component);

    }

    /// <summary>
    /// A base class of all colliders.
    /// </summary>
    public class Collider : Component {
        /// <summary>
        /// The rigidbody the collider is attached to.
        /// </summary>
        public Rigidbody attachedRigidbody;
        /// <summary>
        /// The world space bounding volume of the collider (Read Only).
        /// </summary>
        public readonly Bounds bounds;
        /// <summary>
        /// Contact offset value of this collider.
        /// </summary>
        public float contactOffset;
        /// <summary>
        /// Enabled Colliders will collide with other Colliders, disabled Colliders won't.
        /// </summary>
        public bool enabled;
        /// <summary>
        /// Is the collider a trigger?
        /// </summary>
        public bool isTrigger;
        /// <summary>
        /// The material used by the collider.
        /// </summary>
        public PhysicMaterial material;
        /// <summary>
        /// The shared physic material of this collider.
        /// </summary>
        public PhysicMaterial sharedMaterial;


        /// <summary>
        /// Returns a point on the collider that is closest to a given location.
        /// </summary>
        /// <param name="position">Location you want to find the closest point to.</param>
        /// <returns>Vector3 The point on the collider that is closest to the specified location.</returns>
        public Vector3 ClosestPoint(Vector3 position);
        /// <summary>
        /// The closest point to the bounding box of the attached collider.
        /// </summary>
        public Vector3 ClosestPointOnBounds(Vector3 position);
        /// <summary>
        /// Casts a Ray that ignores all Colliders except this one.
        /// </summary>
        /// <param name="ray">The starting point and direction of the ray.</param>
        /// <param name="hitInfo">If true is returned, hitInfo will contain more information about where the collider was hit.</param>
        /// <param name="maxDistance">The max length of the ray.</param>
        /// <returns></returns>
        public bool Raycast(Ray ray, out RaycastHit hitInfo, float maxDistance);
    }

    /// <summary>
    /// Base class for all entities in Unity Scenes.
    /// </summary>
    public class GameObject : Object {
        /// <summary>
        /// Defines whether the GameObject is active in the Scene.
        /// </summary>
        public bool activeInHierarchy;
        /// <summary>
        /// The local active state of this GameObject. (Read Only)
        /// </summary>
        public readonly bool activeSelf;
        /// <summary>
        /// Gets and sets the GameObject's StaticEditorFlags.
        /// </summary>
        public bool isStatic;
        /// <summary>
        /// The layer the game object is in.
        /// </summary>
        public int layer;
        /// <summary>
        /// Scene that the GameObject is part of.
        /// </summary>
        public SceneManagement.Scene scene;
        /// <summary>
        /// Scene culling mask Unity uses to determine which scene to render the GameObject in.
        /// </summary>
        public ulong sceneCullingMask;
        /// <summary>
        /// The tag of this game object.
        /// </summary>
        public string tag;
        /// <summary>
        /// The Transform attached to this GameObject.
        /// </summary>
        public Transform transform;


        /// <summary>
        /// Adds a component class named className to the game object.
        /// </summary>
        /// <typeparam name="T">The type of Component</typeparam>
        /// <returns></returns>
        public T AddComponent<T>() where T : Component;
        /// <summary>
        /// Calls the method named methodName on every MonoBehaviour in this game object or any of its children.
        /// </summary>
        /// <param name="methodName">Name of coroutine.</param>
        /// <param name="parameter">Optional parameter to pass to the method (can be any value).</param>
        /// <param name="options">Should an error be raised if the method does not exist for a given target object?</param>
        public void BroadcastMessage(string methodName, object parameter = null, SendMessageOptions options = SendMessageOptions.RequireReceiver);
        public void BroadcastMessage(string methodName, SendMessageOptions options);
        /// <summary>
        /// Is this game object tagged with <c>tag</c>?
        /// </summary>
        /// <param name="tag">The tag to compare.</param>
        public bool CompareTag(string tag);
        /// <summary>
        /// Returns the component of Type <c>type</c> if the game object has one attached, null if it doesn't.
        /// </summary>
        /// <summary>
        /// Returns the component of Type <c>type</c> if the game object has one attached, null if it doesn't.
        /// </summary>
        public T GetComponent<T>() where T : Component;
        /// <summary>
        /// Returns the component of Type <c>type</c> in the GameObject or any of its children using depth first search.
        /// </summary>
        /// <returns><c>Component</c> A component of the matching type, if found.</returns>
        public T GetComponentInChildren<T>() where T : Component;
        /// <summary>
        /// Returns the component of Type <c>type</c> in the <c>GameObject</c> or any of its parents.
        /// </summary>
        public T GetComponentInParent<T>() where T : Component;
        /// <summary>
        /// Returns all components of Type <c>type</c> in the <c>GameObject</c>.
        /// </summary>
        public T[] GetComponents<T>() where T : Component;
        /// <summary>
        /// Returns all components of Type type in the <c>GameObject</c> or any of its children.
        /// </summary>
        public T[] GetComponentsInChildren<T>() where T : Component;
        /// <summary>
        /// Returns all components of Type type in the <c>GameObject</c> or any of its parents.
        /// </summary>
        public T[] GetComponentsInParent<T>() where T : Component;
        /// <summary>
        /// Calls the method named methodName on every <c>MonoBehaviour</c> in this game object.
        /// </summary>
        /// <param name="methodName">Name of the method to call.</param>
        /// <param name="value">Optional parameter for the method.</param>
        /// <param name="options">Should an error be raised if the target object doesn't implement the method for the message?</param>
        public void SendMessage(string methodName);
        public void SendMessage(string methodName, object value);
        public void SendMessage(string methodName, object value, SendMessageOptions options);
        public void SendMessage(string methodName, SendMessageOptions options);
        /// <summary>
        /// Calls the method named methodName on every MonoBehaviour in this game object and on every ancestor of the behaviour.
        /// </summary>
        /// <param name="methodName">Name of method to call.</param>
        /// <param name="value">Optional parameter value for the method.</param>
        /// <param name="options">Should an error be raised if the method does not exist on the target object?</param>
        public void SendMessageUpwards(string methodName, SendMessageOptions options);
        public void SendMessageUpwards(string methodName, object value = null, SendMessageOptions options = SendMessageOptions.RequireReceiver);
        /// <summary>
        /// Activates/Deactivates the GameObject, depending on the given <c>true</c> or <c>false</c> value.
        /// </summary>
        /// <param name="value">Activate or deactivate the object, where true activates the <c>GameObject</c> and false deactivates the <c>GameObject</c>.</param>
        public void SetActive(bool value);
        /// <summary>
        /// Gets the component of the specified type, if it exists.
        /// </summary>
        /// <param name="type">The type of the component to retrieve.</param>
        /// <param name="component">The output argument that will contain the component or <c>null</c>.</param>
        /// <returns><c>bool</c> Returns <c>true</c> if the component is found, <c>false</c> otherwise.</returns>
        public bool TryGetComponent(Type type, out Component component);
        public bool TryGetComponent(out T component);


        /// <summary>
        /// Creates a game object with a primitive mesh renderer and appropriate collider.
        /// </summary>
        /// <param name="type">The type of primitive object to create.</param>
        public static GameObject CreatePrimitive(PrimitiveType type);
        /// <summary>
        /// Finds a GameObject by <c>name</c> and returns it.
        /// </summary>
        public static GameObject Find(string name);
        /// <summary>
        /// Returns an array of active GameObjects tagged <c>tag</c>. Returns empty array if no GameObject was found.
        /// </summary>
        /// <param name="tag">The name of the tag to search <c>GameObjects</c> for.</param>
        public static GameObject[] FindGameObjectsWithTag(string tag);
        /// <summary>
        /// Returns one active GameObject tagged <c>tag</c>. Returns <c>null</c> if no GameObject was found.
        /// </summary>
        /// <param name="tag">The tag to search for.</param>
        public static GameObject FindWithTag(string tag);
    }

    /// <summary>
    /// Interface into the Input system.
    /// <para>
    /// Use this class to read the axes set up in the Conventional Game Input, and to access multi-touch/accelerometer data on mobile devices.
    /// </para>
    /// </summary>
    public class Input {
        /// <summary>
        /// Last measured linear acceleration of a device in three-dimensional space. (Read Only)
        /// </summary>
        public static readonly Vector3 acceleration;
        /// <summary>
        /// Number of acceleration measurements which occurred during last frame.
        /// </summary>
        public static int accelerationEventCount;
        /// <summary>
        /// Returns list of acceleration measurements which occurred during the last frame. (Read Only) (Allocates temporary variables).
        /// </summary>
        public static readonly AccelerationEvent[] accelerationEvents;
        /// <summary>
        /// Is any key or mouse button currently held down? (Read Only)
        /// </summary>
        public static readonly bool anyKey;
        /// <summary>
        /// Returns true the first frame the user hits any key or mouse button. (Read Only)
        /// </summary>
        public static readonly bool anyKeyDown;
        /// <summary>
        /// Should Back button quit the application?
        /// </summary>
        public static bool backButtonLeavesApp;
        /// <summary>
        /// Property for accessing compass (handheld devices only). (Read Only)
        /// </summary>
        public static readonly Compass compass;
        /// <summary>
        /// This property controls if input sensors should be compensated for screen orientation.
        /// </summary>
        public static bool compensateSensors;
        /// <summary>
        /// The current text input position used by IMEs to open windows.
        /// </summary>
        public static Vector2 compositionCursorPos;
        /// <summary>
        /// The current IME composition string being typed by the user.
        /// </summary>
        public static string compositionString;
        /// <summary>
        /// Device physical orientation as reported by OS. (Read Only)
        /// </summary>
        public static readonly DeviceOrientation deviceOrientation;
        /// <summary>
        /// Returns default gyroscope.
        /// </summary>
        public static Gyroscope gyro;
        /// <summary>
        /// Controls enabling and disabling of IME input composition.
        /// </summary>
        public static IMECompositionMode imeCompositionMode;
        /// <summary>
        /// Does the user have an IME keyboard input source selected?
        /// </summary>
        public static bool imeIsSelected;
        /// <summary>
        /// Returns the keyboard input entered this frame. (Read Only)
        /// </summary>
        public static readonly string inputString;
        /// <summary>
        /// Property for accessing device location (handheld devices only). (Read Only)
        /// </summary>
        public static readonly LocationService location;
        /// <summary>
        /// The current mouse position in pixel coordinates. (Read Only)
        /// </summary>
        public static readonly Vector3 mousePosition;
        /// <summary>
        /// Indicates if a mouse device is detected.
        /// </summary>
        public static bool mousePresent;
        /// <summary>
        /// The current mouse scroll delta. (Read Only)
        /// </summary>
        public static readonly Vector2 mouseScrollDelta;
        /// <summary>
        /// Property indicating whether the system handles multiple touches.
        /// </summary>
        public static bool multiTouchEnabled;
        /// <summary>
        /// Enables/Disables mouse simulation with touches. By default this option is enabled.
        /// </summary>
        public static bool simulateMouseWithTouches;
        /// <summary>
        /// Returns true when Stylus Touch is supported by a device or platform.
        /// </summary>
        public static bool stylusTouchSupported;
        /// <summary>
        /// Number of touches. Guaranteed not to change throughout the frame. (Read Only)
        /// </summary>
        public static readonly int touchCount;
        /// <summary>
        /// Returns list of objects representing status of all touches during last frame. (Read Only) (Allocates temporary variables).
        /// </summary>
        public static readonly Touch[] touches;
        /// <summary>
        /// Bool value which let's users check if touch pressure is supported.
        /// </summary>
        public static bool touchPressureSupported;
        /// <summary>
        /// Returns whether the device on which application is currently running supports touch input.
        /// </summary>
        public static bool touchSupported;


        /// <summary>
        /// Returns specific acceleration measurement which occurred during last frame. (Does not allocate temporary variables).
        /// </summary>
        public static AccelerationEvent GetAccelerationEvent(int index);
        /// <summary>
        /// Returns the value of the virtual axis identified by <p>axisName</p>.
        /// </summary>
        public static float GetAxis(string axisName);
        /// <summary>
        /// Returns the value of the virtual axis identified by <p>axisName</p> with no smoothing filtering applied.
        /// </summary>
        public static float GetAxisRaw(string axisName);
        /// <summary>
        /// Returns true while the virtual button identified by <p>buttonName</p> is held down.
        /// </summary>
        /// <param name="buttonName">The name of the button such as Jump.</param>
        /// <returns>bool True when an axis has been pressed and not released.</returns>
        public static bool GetButton(string buttonName);
        /// <summary>
        /// Returns true during the frame the user pressed down the virtual button identified by <p>buttonName</p>.
        /// </summary>
        /// <param name="buttonName">The name of the button such as Jump.</param>
        public static bool GetButtonDown(string buttonName);
        /// <summary>
        /// Returns true the first frame the user releases the virtual button identified by <p>buttonName</p>.
        /// </summary>
        /// <param name="buttonName">The name of the button such as Jump.</param>
        public static bool GetButtonUp(string buttonName);
        /// <summary>
        /// Retrieves a list of input device names corresponding to the index of an Axis configured within Input Manager.
        /// </summary>
        /// <returns>string[] Returns an array of joystick and gamepad device names.</returns>
        public static string[] GetJoystickNames();
        /// <summary>
        /// Returns true while the user holds down the key identified by <p>name</p>.
        /// </summary>
        public static bool GetKey(string name);
        /// <summary>
        /// Returns true during the frame the user starts pressing down the key identified by <p>name</p>.
        /// </summary>
        public static bool GetKeyDown(string name);
        /// <summary>
        /// Returns true during the frame the user releases the key identified by <p>name</p>.
        /// </summary>
        public static bool GetKeyUp(string name);
        /// <summary>
        /// Returns whether the given mouse button is held down.
        /// </summary>
        public static bool GetMouseButton(int button);
        /// <summary>
        /// Returns true during the frame the user pressed the given mouse button.
        /// </summary>
        public static bool GetMouseButtonDown(int button);
        /// <summary>
        /// Returns true during the frame the user releases the given mouse button.
        /// </summary>
        public static bool GetMouseButtonUp(int button);
        /// <summary>
        /// Call <c>Input.GetTouch</c> to obtain a <c>Touch</c> struct.
        /// </summary>
        /// <param name="index">The touch input on the device screen.</param>
        /// <returns>Touch Touch details in the struct.</returns>
        public static Touch GetTouch(int index);
        /// <summary>
        /// Determine whether a particular joystick model has been preconfigured by Unity. (Linux-only).
        /// </summary>
        /// <param name="joystickName">The name of the joystick to check (returned by Input.GetJoystickNames).</param>
        /// <returns>bool True if the joystick layout has been preconfigured; false otherwise.</returns>
        public static bool IsJoystickPreconfigured(string joystickName);
        /// <summary>
        /// Resets all input. After ResetInputAxes all axes return to 0 and all buttons return to 0 for one frame.
        /// </summary>
        public static void ResetInputAxes();

    }

    /// <summary>
    /// The material class.
    /// <para>
    /// This class exposes all properties from a material, allowing you to animate them.
    /// You can also use it to set custom shader properties that can't be accessed through the inspector (e.g. matrices).
    /// </para>
    /// In order to get the material used by an object, use the <c>Renderer.material</c> property.
    /// </summary>
    public class Material : Object {
        /// <summary>
        /// The main color of the Material.
        /// </summary>
        public Color color;
        /// <summary>
        /// Gets and sets whether the Double Sided Global Illumination setting is enabled for this material.
        /// </summary>
        public bool doubleSidedGI;
        /// <summary>
        /// Gets and sets whether GPU instancing is enabled for this material.
        /// </summary>
        public bool enableInstancing;
        /// <summary>
        /// Defines how the material should interact with lightmaps and lightprobes.
        /// </summary>
        public MaterialGlobalIlluminationFlags globalIlluminationFlags;
        /// <summary>
        /// The main texture.
        /// </summary>
        public Texture mainTexture;
        /// <summary>
        /// The offset of the main texture.
        /// </summary>
        public Vector2 mainTextureOffset;
        /// <summary>
        /// The scale of the main texture.
        /// </summary>
        public Vector2 mainTextureScale;
        /// <summary>
        /// How many passes are in this material (Read Only).
        /// </summary>
        public readonly int passCount;
        /// <summary>
        /// Render queue of this material.
        /// </summary>
        public int renderQueue;
        /// <summary>
        /// The shader used by the material.
        /// </summary>
        public Shader shader;
        /// <summary>
        /// Additional shader keywords set by this material.
        /// </summary>
        public string[] shaderKeywords;


        /// <summary>
        /// Create a temporary Material.
        /// </summary>
        /// <param name="shader">Create a material with a given Shader.</param>
        /// <param name="source">Create a material by copying all properties from another material.</param>
        public Material(Shader shader);
        public Material(Material source);


        /// <summary>
        /// Computes a CRC hash value from the content of the material.
        /// </summary>
        public int ComputeCRC();
        /// <summary>
        /// Copy properties from other material into this material.
        /// </summary>
        public void CopyPropertiesFromMaterial(Material mat);
        /// <summary>
        /// Unset a shader keyword.
        /// </summary>
        public void DisableKeyword(string keyword);
        /// <summary>
        /// Sets a shader keyword that is enabled by this material.
        /// </summary>
        public void EnableKeyword(string keyword);
        /// <summary>
        /// Returns the index of the pass <c>passName</c>.
        /// </summary>
        public int FindPass(string passName);
        /// <summary>
        /// Get a named color value.
        /// </summary>
        /// <param name="name">The name of the property.</param>
        /// <param name="nameID">The name ID of the property retrieved by <c>Shader.PropertyToID</c>.</param>
        public Color GetColor(string name);
        public Color GetColor(int nameID);
        /// <summary>
        /// Get a named color array.
        /// </summary>
        /// <param name="name">The name of the property.</param>
        /// <param name="nameID">The name ID of the property retrieved by <c>Shader.PropertyToID</c>.</param>
        public Color[] GetColorArray(string name);
        public Color[] GetColorArray(int nameID);
        /// <summary>
        /// Get a named float value.
        /// </summary>
        /// <param name="name">The name of the property.</param>
        /// <param name="nameID">The name ID of the property retrieved by <c>Shader.PropertyToID</c>.</param>
        public float GetFloat(string name);
        public float GetFloat(int nameID);
        /// <summary>
        /// Get a named float array.
        /// </summary>
        /// <param name="name">The name of the property.</param>
        /// <param name="nameID">The name ID of the property retrieved by <c>Shader.PropertyToID</c>.</param>
        public float[] GetFloatArray(string name);
        public float[] GetFloatArray(int nameID);
        /// <summary>
        /// Get a named integer value.
        /// </summary>
        /// <param name="name">The name of the property.</param>
        /// <param name="nameID">The name ID of the property retrieved by <c>Shader.PropertyToID</c>.</param>
        public int GetInt(string name);
        public int GetInt(int nameID);
        /// <summary>
        /// Get a named matrix value from the shader.
        /// </summary>
        /// <param name="name">The name of the property.</param>
        /// <param name="nameID">The name ID of the property retrieved by <c>Shader.PropertyToID</c>.</param>
        public Matrix4x4 GetMatrix(string name);
        public Matrix4x4 GetMatrix(int nameID);
        /// <summary>
        /// Get a named matrix array.
        /// </summary>
        /// <param name="name">The name of the property.</param>
        /// <param name="nameID">The name ID of the property retrieved by <c>Shader.PropertyToID</c>.</param>
        public Matrix4x4[] GetMatrixArray(string name);
        public Matrix4x4[] GetMatrixArray(int nameID);
        /// <summary>
        /// Returns the name of the shader pass at index <c>pass</c>.
        /// </summary>
        public string GetPassName(int pass);
        /// <summary>
        /// bool True if the Shader pass is enabled.
        /// </summary>
        /// <param name="passName">Shader pass name (case insensitive).</param>
        public bool GetShaderPassEnabled(string passName);
        /// <summary>
        /// Get the value of material's shader tag.
        /// </summary>
        public string GetTag(string tag, bool searchFallbacks);
        public string GetTag(string tag, bool searchFallbacks, string defaultValue);
        /// <summary>
        /// Get a named texture.
        /// </summary>
        /// <param name="name">The name of the property.</param>
        /// <param name="nameID">The name ID of the property retrieved by <c>Shader.PropertyToID</c>.</param>
        public Texture GetTexture(string name);
        public Texture GetTexture(int nameID);
        /// <summary>
        /// Gets the placement offset of texture <c>propertyName</c>.
        /// </summary>
        /// <param name="name">The name of the property.</param>
        /// <param name="nameID">The name ID of the property retrieved by <c>Shader.PropertyToID</c>.</param>
        public Vector2 GetTextureOffset(string name);
        public Vector2 GetTextureOffset(int nameID);
        /// <summary>
        /// <c>int[]</c> IDs of all texture properties exposed on this material.
        /// </summary>
        /// <param name="outNames">IDs of all texture properties exposed on this material.</param>
        public int[] GetTexturePropertyNameIDs();
        public void GetTexturePropertyNameIDs(List<int> outNames);
        /// <summary>
        /// <c>string[]</c> Names of all texture properties exposed on this material.
        /// </summary>
        /// <param name="outNames">Names of all texture properties exposed on this material.</param>
        public string[] GetTexturePropertyNames();
        public void GetTexturePropertyNames(List<string> outNames);
        /// <summary>
        /// Gets the placement scale of texture <c>propertyName</c>.
        /// </summary>
        /// <param name="name">The name of the property.</param>
        /// <param name="nameID">The name ID of the property retrieved by <c>Shader.PropertyToID</c>.</param>
        public Vector2 GetTextureScale(string name);
        public Vector2 GetTextureScale(int nameID);
        /// <summary>
        /// Get a named vector array.
        /// </summary>
        /// <param name="name">The name of the property.</param>
        /// <param name="nameID">The name ID of the property retrieved by <c>Shader.PropertyToID</c>.</param>
        public Vector4[] GetVectorArray(string name);
        public Vector4[] GetVectorArray(int nameID);
        /// <summary>
        /// Checks if material's shader has a property of a given name.
        /// </summary>
        /// <param name="name">The name of the property.</param>
        /// <param name="nameID">The name ID of the property retrieved by <c>Shader.PropertyToID</c>.</param>
        public bool HasProperty(string name);
        public bool HasProperty(int nameID);
        /// <summary>
        /// Is the shader keyword enabled on this material?
        /// </summary>
        public bool IsKeywordEnabled(string keyword);
        /// <summary>
        /// Interpolate properties between two materials.
        /// </summary>
        public void Lerp(Material start, Material end, float t);
        /// <summary>
        /// Sets a named <c>ComputeBuffer</c> value.
        /// </summary>
        /// <param name="name">Property name ID, use <c>Shader.PropertyToID</c> to get it.</param>
        /// <param name="name">Property name.</param>
        /// <param name="value">The <c>ComputeBuffer</c> value to set.</param>
        public void SetBuffer(string name, ComputeBuffer value);
        public void SetBuffer(int nameID, ComputeBuffer value);
        /// <summary>
        /// Sets a named color value.
        /// </summary>
        /// <param name="name">Property name, e.g. "_Color".</param>
        /// <param name="nameID">Property name ID, use <c>Shader.PropertyToID</c> to get it.</param>
        /// <param name="value">Color value to set.</param>
        public void SetColor(string name, Color value);
        public void SetColor(int nameID, Color value);
        /// <summary>
        /// Sets a color array property.
        /// </summary>
        /// <param name="name">Property name.</param>
        /// <param name="nameID">Property name ID, use <c>Shader.PropertyToID</c> to get it.</param>
        /// <param name="values">Array of values to set.</param>
        public void SetColorArray(string name, Color[] values);
        public void SetColorArray(int nameID, Color[] values);
        public void SetColorArray(string name, List<Color> values);
        public void SetColorArray(int nameID, List<Color> values);
        /// <summary>
        /// Sets a ComputeBuffer as a named constant buffer for the material.
        /// </summary>
        /// <param name="name">The name of the constant buffer to override.</param>
        /// <param name="value">The ComputeBuffer to override the constant buffer values with, or null to remove binding.</param>
        /// <param name="offset">Offset in bytes from the beginning of the ComputeBuffer to bind. Must be a multiple of SystemInfo.MinConstantBufferAlignment, or 0 if that value is 0.</param>
        /// <param name="size">The number of bytes to bind.</param>
        /// <param name="nameID">The shader property ID of the constant buffer to override.</param>
        public void SetConstantBuffer(string name, ComputeBuffer value, int offset, int size);
        public void SetConstantBuffer(int nameID, ComputeBuffer value, int offset, int size);
        /// <summary>
        /// Sets a named float value.
        /// </summary>
        /// <param name="name">Property name, e.g. "_Glossiness".</param>
        /// <param name="nameID">Property name ID, use <c>Shader.PropertyToID</c> to get it.</param>
        /// <param name="value">Float value to set.</param>
        public void SetFloat(string name, float value);
        public void SetFloat(int nameID, float value);
        /// <summary>
        /// Sets a float array property.
        /// </summary>
        /// <param name="name">Property name.</param>
        /// <param name="nameID">Property name ID. Use <c>Shader.PropertyToID</c> to get this ID.</param>
        /// <param name="values">Array of values to set.</param>
        public void SetFloatArray(string name, float[] values);
        public void SetFloatArray(int nameID, float[] values);
        public void SetFloatArray(string name, List<float> values);
        public void SetFloatArray(int nameID, List<float> values);
        /// <summary>
        /// Sets a named integer value.
        /// </summary>
        /// <param name="name">Property name, e.g. "_SrcBlend".</param>
        /// <param name="nameID">Property name ID, use <c>Shader.PropertyToID</c> to get it.</param>
        /// <param name="value">Integer value to set.</param>
        public void SetInt(string name, int value);
        public void SetInt(int nameID, int value);
        /// <summary>
        /// Sets a named matrix for the shader.
        /// </summary>
        /// <param name="name">Property name, e.g. "_CubemapRotation".</param>
        /// <param name="nameID">Property name ID, use <c>Shader.PropertyToID</c> to get it.</param>
        /// <param name="value">Matrix value to set.</param>
        public void SetMatrix(string name, Matrix4x4 value);
        public void SetMatrix(int nameID, Matrix4x4 value);
        /// <summary>
        /// Sets a matrix array property on the material. If a matrix array property with the given name already exists, the old value is replaced.
        /// </summary>
        /// <param name="name">Property name.</param>
        /// <param name="nameID">Property name ID, use <c>Shader.PropertyToID</c> to get it.</param>
        /// <param name="values">Array of values to set.</param>
        public void SetMatrixArray(string name, Matrix4x4[] values);
        public void SetMatrixArray(int nameID, Matrix4x4[] values);
        public void SetMatrixArray(string name, List<Matrix4x4> values);
        public void SetMatrixArray(int nameID, List<Matrix4x4> values);
        /// <summary>
        /// Sets an override tag/value on the material.
        /// </summary>
        /// <param name="tag">Name of the tag to set.</param>
        /// <param name="val">Name of the value to set. Empty string to clear the override flag.</param>
        public void SetOverrideTag(string tag, string val);
        /// <summary>
        /// bool If false is returned, no rendering should be done.
        /// </summary>
        /// <param name="pass">Shader pass number to setup.</param>
        public bool SetPass(int pass);
        /// <summary>
        /// Enables or disables a Shader pass on a per-Material level.
        /// </summary>
        /// <param name="passName">Shader pass name (case insensitive).</param>
        /// <param name="enabled">Flag indicating whether this Shader pass should be enabled.</param>
        public void SetShaderPassEnabled(string passName, bool enabled);
        /// <summary>
        /// Sets a named texture.
        /// </summary>
        /// <param name="name">Property name, e.g. "_MainTex".</param>
        /// <param name="nameID">Property name ID, use <c>Shader.PropertyToID</c> to get it.</param>
        /// <param name="value">Texture to set.</param>
        /// <param name="element">Optional parameter that specifies the type of data from the render texture to set.</param>
        public void SetTexture(string name, Texture value);
        public void SetTexture(int nameID, Texture value);
        public void SetTexture(string name, RenderTexture value, Rendering.RenderTextureSubElement element);
        public void SetTexture(int nameID, RenderTexture value, Rendering.RenderTextureSubElement element);
        /// <summary>
        /// Sets the placement offset of texture <c>propertyName</c>.
        /// </summary>
        /// <param name="name">Property name, for example: "_MainTex".</param>
        /// <param name="nameID">Property name ID, use <c>Shader.PropertyToID</c> to get it.</param>
        /// <param name="value">Texture placement offset.</param>
        public void SetTextureOffset(string name, Vector2 value);
        public void SetTextureOffset(int nameID, Vector2 value);
        /// <summary>
        /// Sets the placement scale of texture <c>propertyName</c>.
        /// </summary>
        /// <param name="name">Property name, e.g. "_MainTex".</param>
        /// <param name="nameID">Property name ID, use <c>Shader.PropertyToID</c> to get it.</param>
        /// <param name="value">Texture placement scale.</param>
        public void SetTextureScale(string name, Vector2 value);
        public void SetTextureScale(int nameID, Vector2 value);
        /// <summary>
        /// Sets a named vector value.
        /// </summary>
        /// <param name="name">Property name, e.g. "_WaveAndDistance".</param>
        /// <param name="nameID">Property name ID, use <c>Shader.PropertyToID</c> to get it.</param>
        /// <param name="value">Vector value to set.</param>
        public void SetVector(string name, Vector4 value);
        public void SetVector(int nameID, Vector4 value);
        /// <summary>
        /// Sets a vector array property.
        /// </summary>
        /// <param name="name">Property name.</param>
        /// <param name="nameID">Property name ID, use <c>Shader.PropertyToID</c> to get it.</param>
        /// <param name="values">Array of values to set.</param>
        public void SetVectorArray(string name, Vector4[] values);
        public void SetVectorArray(int nameID, Vector4[] values);
        public void SetVectorArray(string name, List<Vector4> values);
        public void SetVectorArray(int nameID, List<Vector4> values);
    }

    /// <summary>
    /// A mesh collider allows you to do collision detection between meshes and primitives.
    /// </summary>
    public class MeshCollider : Collider {
        /// <summary>
        /// Use a convex collider from the mesh.
        /// </summary>
        public bool convex;
        /// <summary>
        /// Options used to enable or disable certain features in mesh cooking.
        /// </summary>
        public MeshColliderCookingOptions cookingOptions;
        /// <summary>
        /// The mesh object used for collision detection.
        /// </summary>
        public Mesh sharedMesh;
    }

    /// <summary>
    /// MonoBehaviour is the base class from which every Unity script derives.
    /// <para>
    /// When you use C#, you must explicitly derive from MonoBehaviour.
    /// </para>
    /// This class doesn't support the null-conditional operator (?.) and the null-coalescing operator (??).
    /// </summary>
    public class MonoBehaviour : Behaviour {
        /// <summary>
        /// Allow a specific instance of a MonoBehaviour to run in edit mode (only available in the editor).
        /// </summary>
        public bool runInEditMode;
        /// <summary>
        /// Disabling this lets you skip the GUI layout phase.
        /// </summary>
        public bool useGUILayout;


        /// <summary>
        /// Cancels all Invoke calls on this MonoBehaviour.
        /// </summary>
        public void CancelInvoke();
        /// <summary>
        /// Invokes the method <c>methodName</c> in time seconds.
        /// </summary>
        /// <param name="methodName">Name of coroutine.</param>
        /// <param name="time">Delay time in seconds.</param>
        public void Invoke(string methodName, float time);
        /// <summary>
        /// Invokes the method <c>methodName</c> in <c>time</c> seconds, then repeatedly every <c>repeatRate</c> seconds.
        /// </summary>
        /// <param name="methodName">Name of coroutine.</param>
        /// <param name="time">Delay time in seconds.</param>
        /// <param name="repeatRate">Repeat rate in seconds</param>
        public void InvokeRepeating(string methodName, float time, float repeatRate);
        /// <summary>
        /// Is any invoke on methodName pending?
        /// </summary>
        /// <param name="methodName">Name of coroutine.</param>
        public bool IsInvoking(string methodName);
        /// <summary>
        /// Starts a Coroutine.
        /// </summary>
        public Coroutine StartCoroutine(IEnumerator routine);
        /// <summary>
        /// Stops all coroutines running on this behaviour.
        /// </summary>
        public void StopAllCoroutines();
        /// <summary>Stops the first coroutine named methodName, or the coroutine stored in routine running on this behaviour.</summary>
        /// <param name="methodName">Name of coroutine.</param>
        /// <param name="routine">Name of the function in code, including coroutines.</param>
        public void StopCoroutine(string methodName);
        public void StopCoroutine(IEnumerator routine);
        public void StopCoroutine(Coroutine routine);


        /// <summary>
        /// Logs message to the Unity Console (identical to Debug.Log).
        /// </summary>
        public static void print(object message);
    }

    /// <summary>
    /// Base class for all objects Unity can reference.
    /// <para>
    /// Any public variable you make that derives from Object gets shown in the inspector as a drop target,
    /// allowing you to set the value from the GUI. UnityEngine.Object is the base class of all built-in Unity objects.
    /// </para>
    /// </summary>
    public class Object {
        /// <summary>
        /// Should the object be hidden, saved with the Scene or modifiable by the user?
        /// </summary>
        public HideFlags hideFlags;
        /// <summary>
        /// The name of the object.
        /// </summary>
        public string name;


        /// <summary>
        /// Returns the instance id of the object.
        /// </summary>
        public int GetInstanceID();
        /// <summary>
        /// <c>string</c> The name returned by ToString.
        /// </summary>
        public string ToString();

        /// <summary>
        /// Removes a GameObject, component or asset.
        /// </summary>
        /// <param name="obj">The object to destroy.</param>
        /// <param name="t">The optional amount of time to delay before destroying the object.</param>
        public static void Destroy(Object obj, float t = 0.0F);
        /// <summary>
        /// Destroys the object <c>obj</c> immediately. You are strongly recommended to use Destroy instead.
        /// </summary>
        /// <param name="obj">Object to be destroyed.</param>
        /// <param name="allowDestroyingAssets">Set to true to allow assets to be destroyed.</param>
        public static void DestroyImmediate(Object obj, bool allowDestroyingAssets = false);
        /// <summary>
        /// Do not destroy the target Object when loading a new Scene.
        /// </summary>
        /// <param name="target">An Object not destroyed on Scene change.</param>
        public static void DontDestroyOnLoad(Object target);
        /// <summary>
        /// Returns the first active loaded object of Type <c>type.</c>
        /// </summary>
        /// <returns><c>T</c> This returns the Object that matches the specified type. It returns null if no Object matches the type.</returns>
        public static T FindObjectOfType();
        /// <summary>
        /// Returns a list of all active loaded objects of Type <c>type.</c>
        /// </summary>
        /// <returns><c>T[]</c> The array of objects found matching the type specified.</returns>
        public static T[] FindObjectsOfType();
        /// <summary>
        /// Clones the object original and returns the clone.</c>
        /// </summary>
        /// <returns><c>Object</c> The instantiated clone.</returns>
        /// <param name="original">An existing object that you want to make a copy of.</param>
        /// <param name="position">Position for the new object.</param>
        /// <param name="rotation">Orientation of the new object.</param>
        /// <param name="parent">Parent that will be assigned to the new object.</param>
        /// <param name="instantiateInWorldSpace">When you assign a parent Object, pass true to position the new object directly in world space. Pass false to set the Object’s position relative to its new parent.</param>
        public static Object Instantiate(Object original);
        public static Object Instantiate(Object original, Transform parent);
        public static Object Instantiate(Object original, Transform parent, bool instantiateInWorldSpace);
        public static Object Instantiate(Object original, Vector3 position, Quaternion rotation);
        public static Object Instantiate(Object original, Vector3 position, Quaternion rotation, Transform parent);


        /// <summary>
        /// Does the object exist?
        /// </summary>
        public static bool operator !(Object obj);
        /// <summary>
        /// Compares if two objects refer to a different object.
        /// </summary>
        public static bool operator !=(Object x, Object y);
        /// <summary>
        /// Compares two object references to see if they refer to the same object.
        /// </summary>
        /// <param name="x">The first Object.</param>
        /// <param name="y">The Object to compare against the first.</param>
        public static bool operator ==(Object x, Object y);
    }

    /// <summary>
    /// Physics material describes how to handle colliding objects (friction, bounciness).
    /// </summary>
    public class PhysicMaterial : Object {
        /// <summary>
        /// Determines how the bounciness is combined.
        /// </summary>
        public PhysicMaterialCombine bounceCombine;
        /// <summary>
        /// How bouncy is the surface? A value of 0 will not bounce. A value of 1 will bounce without any loss of energy.
        /// </summary>
        public float bounciness;
        /// <summary>
        /// The friction used when already moving. This value is usually between 0 and 1.
        /// </summary>
        public float dynamicFriction;
        /// <summary>
        /// Determines how the friction is combined.
        /// </summary>
        public PhysicMaterialCombine frictionCombine;
        /// <summary>
        /// The friction coefficient used when an object is lying on a surface.
        /// </summary>
        public float staticFriction;


        /// <summary>
        /// Creates a new material.
        /// </summary>
        public PhysicMaterial();
    }

    /// <summary>
    /// General functionality for all renderers.
    /// <para>
    /// A renderer is what makes an object appear on the screen.
    /// Use this class to access the renderer of any object, mesh or Particle System.
    /// Renderers can be disabled to make objects invisible, and the materials can be accessed and modified through them.
    /// </para>
    /// </summary>
    public class Renderer : Component {
        /// <summary>
        /// Controls if dynamic occlusion culling should be performed for this renderer.
        /// </summary>
        public bool allowOcclusionWhenDynamic;
        /// <summary>
        /// The bounding volume of the renderer (Read Only).
        /// </summary>
        public readonly Bounds bounds;
        /// <summary>
        /// Makes the rendered 3D object visible if enabled.
        /// </summary>
        public bool enabled;
        /// <summary>
        /// Allows turning off rendering for a specific component.
        /// </summary>
        public bool forceRenderingOff;
        /// <summary>
        /// Has this renderer been statically batched with any other renderers?
        /// </summary>
        public bool isPartOfStaticBatch;
        /// <summary>
        /// Is this renderer visible in any camera? (Read Only)
        /// </summary>
        public readonly bool isVisible;
        /// <summary>
        /// The index of the baked lightmap applied to this renderer.
        /// </summary>
        public int lightmapIndex;
        /// <summary>
        /// public Vector4 lightmapScaleOffset;
        /// </summary>
        public Vector4 lightmapScaleOffset;
        /// <summary>
        /// If set, the Renderer will use the Light Probe Proxy Volume component attached to the source <c>GameObject</c>.
        /// </summary>
        public GameObject lightProbeProxyVolumeOverride;
        /// <summary>
        /// The light probe interpolation type.
        /// </summary>
        public Rendering.LightProbeUsage lightProbeUsage;
        /// <summary>
        /// Matrix that transforms a point from local space into world space (Read Only).
        /// </summary>
        public readonly Matrix4x4 localToWorldMatrix;
        /// <summary>
        /// Returns the first instantiated Material assigned to the renderer.
        /// </summary>
        public Material material;
        /// <summary>
        /// Returns all the instantiated materials of this object.
        /// </summary>
        public Material[] materials;
        /// <summary>
        /// Specifies the mode for motion vector rendering.
        /// </summary>
        public MotionVectorGenerationMode motionVectorGenerationMode;
        /// <summary>
        /// If set, Renderer will use this Transform's position to find the light or reflection probe.
        /// </summary>
        public Transform probeAnchor;
        /// <summary>
        /// Describes how this renderer is updated for ray tracing.
        /// </summary>
        public Experimental.Rendering.RayTracingMode rayTracingMode;
        /// <summary>
        /// The index of the realtime lightmap applied to this renderer.
        /// </summary>
        public int realtimeLightmapIndex;
        /// <summary>
        /// The UV scale and offset used for a realtime lightmap.
        /// </summary>
        public Vector4 realtimeLightmapScaleOffset;
        /// <summary>
        /// Does this object receive shadows?
        /// </summary>
        public bool receiveShadows;
        /// <summary>
        /// public Rendering.ReflectionProbeUsage reflectionProbeUsage;
        /// </summary>
        public Rendering.ReflectionProbeUsage reflectionProbeUsage;
        /// <summary>
        /// This value sorts renderers by priority. Lower values are rendered first and higher values are rendered last.
        /// </summary>
        public int rendererPriority;
        /// <summary>
        /// Determines which rendering layer this renderer lives on.
        /// </summary>
        public uint renderingLayerMask;
        /// <summary>
        /// Does this object cast shadows?
        /// </summary>
        public Rendering.ShadowCastingMode shadowCastingMode;
        /// <summary>
        /// The shared material of this object.
        /// </summary>
        public Material sharedMaterial;
        /// <summary>
        /// All the shared materials of this object.
        /// </summary>
        public Material[] sharedMaterials;
        /// <summary>
        /// Unique ID of the Renderer's sorting layer.
        /// </summary>
        public int sortingLayerID;
        /// <summary>
        /// Name of the Renderer's sorting layer.
        /// </summary>
        public string sortingLayerName;
        /// <summary>
        /// Renderer's order within a sorting layer.
        /// </summary>
        public int sortingOrder;
        /// <summary>
        /// Matrix that transforms a point from world space into local space (Read Only).
        /// </summary>
        public readonly Matrix4x4 worldToLocalMatrix;


        /// <summary>
        /// Returns an array of closest reflection probes with weights, weight shows how much influence the probe has on the renderer, this value is also used when blending between reflection probes occur.
        /// </summary>
        public void GetClosestReflectionProbes(List<ReflectionProbeBlendInfo> result);
        /// <summary>
        /// Returns all the instantiated materials of this object.
        /// </summary>
        /// <param name="m">A list of materials to populate.</param>
        public void GetMaterials(List<Material> m);
        /// <summary>
        /// Get per-Renderer or per-Material property block.
        /// </summary>
        /// <param name="properties">Material parameters to retrieve.</param>
        /// <param name="materialIndex">The index of the Material you want to get overridden parameters from. The index ranges from 0 to Renderer.sharedMaterials.Length-1.</param>
        public void GetPropertyBlock(MaterialPropertyBlock properties);
        public void GetPropertyBlock(MaterialPropertyBlock properties, int materialIndex);
        /// <summary>
        /// Returns all the shared materials of this object.
        /// </summary>
        /// <param name="m">A list of materials to populate.</param>
        public void GetSharedMaterials(List<Material> m);
        /// <summary>
        /// Returns true if the Renderer has a material property block attached via SetPropertyBlock.
        /// </summary>
        public bool HasPropertyBlock();
        /// <summary>
        /// Lets you set or clear per-renderer or per-material parameter overrides.
        /// </summary>
        /// <param name="properties">Property block with values you want to override.</param>
        /// <param name="materialIndex">The index of the Material you want to override the parameters of. The index ranges from 0 to Renderer.sharedMaterial.Length-1.</param>
        public void SetPropertyBlock(MaterialPropertyBlock properties);
        public void SetPropertyBlock(MaterialPropertyBlock properties, int materialIndex);
    }

    /// <summary>
    /// The Resources class allows you to find and access Objects including assets.
    /// <para>
    /// In the editor, <c>Resources.FindObjectsOfTypeAll</c> can be used to locate assets and Scene objects.
    /// </para>
    /// <para>
    /// All assets that are in a folder named "Resources" anywhere in the Assets folder can be accessed via the <c>Resources.Load</c> functions.
    /// Multiple "Resources" folders may exist and when loading objects each will be examined.
    /// </para>
    /// </summary>
    public class Resources {
        /// <summary>
        /// Returns a list of all objects of Type <p>T</p>.
        /// </summary>
        public static T[] FindObjectsOfTypeAll();
        /// <summary>
        /// Loads an asset stored at <p>path</p> in a folder called Resources.
        /// </summary>
        /// <param name="path">Path to the target resource to load.</param>
        /// <returns>T The requested asset's Type.</returns>
        public static T Load(string path);
        /// <summary>
        /// Loads all assets in a folder or file at <p>path</p> in a Resources folder.
        /// </summary>
        /// <param name="path">
        /// Pathname of the target folder.
        /// When using the empty string (i.e., ""), the function will load the entire contents of the Resources folder.
        /// </param>
        /// <param name="systemTypeInstance">Type filter for objects returned.</param>
        public static Object[] LoadAll(string path);
        public static Object[] LoadAll(string path, Type systemTypeInstance);
        /// <summary>
        /// Asynchronously loads an asset stored at <p>path</p> in a Resources folder.
        /// </summary>
        /// <param name="path">
        /// Pathname of the target folder.
        /// When using the empty string (i.e., ""), the function will load the entire contents of the Resources folder.
        /// </param>
        /// <param name="systemTypeInstance">Type filter for objects returned.</param>
        public static ResourceRequest LoadAsync(string path);
        public static ResourceRequest LoadAsync(string path, Type type);
        /// <summary>
        /// Unloads <p>assetToUnload</p> from memory.
        /// </summary>
        public static void UnloadAsset(Object assetToUnload);
        /// <summary>
        /// AsyncOperation Object on which you can yield to wait until the operation completes.
        /// </summary>
        public static AsyncOperation UnloadUnusedAssets();
    }

    /// <summary>
    /// Control of an object's position through physics simulation.
    /// <para>
    /// Adding a Rigidbody component to an object will put its motion under the control of Unity's physics engine.
    /// Even without adding any code, a Rigidbody object will be pulled downward by gravity and will react to collisions with incoming objects if the right Collider component is also present.
    /// </para>
    /// The Rigidbody also has a scripting API that lets you apply forces to the object and control it in a physically realistic way.
    /// For example, a car's behaviour can be specified in terms of the forces applied by the wheels.
    /// Given this information, the physics engine can handle most other aspects of the car's motion, so it will accelerate realistically and respond correctly to collisions.
    /// </summary>
    public class Rigidbody : Component {
        /// <summary>
        ///  The angular drag of the object.
        /// </summary>
        public float angularDrag;
        /// <summary>
        /// The angular velocity vector of the rigidbody measured in radians per second.
        /// </summary>
        public Vector3 angularVelocity;
        /// <summary>
        /// The center of mass relative to the transform's origin.
        /// </summary>
        public Vector3 centerOfMass;
        /// <summary>
        /// The Rigidbody's collision detection mode.
        /// </summary>
        public CollisionDetectionMode collisionDetectionMode;
        /// <summary>
        /// Controls which degrees of freedom are allowed for the simulation of this Rigidbody.
        /// </summary>
        public RigidbodyConstraints constraints;
        /// <summary>
        /// Should collision detection be enabled? (By default always enabled).
        /// </summary>
        public bool detectCollisions;
        /// <summary>
        /// The drag of the object.
        /// </summary>
        public float drag;
        /// <summary>
        /// Controls whether physics will change the rotation of the object.
        /// </summary>
        public bool freezeRotation;
        /// <summary>
        /// The diagonal inertia tensor of mass relative to the center of mass.
        /// </summary>
        public Vector3 inertiaTensor;
        /// <summary>
        /// The rotation of the inertia tensor.
        /// </summary>
        public Quaternion inertiaTensorRotation;
        /// <summary>
        /// Interpolation allows you to smooth out the effect of running physics at a fixed frame rate.
        /// </summary>
        public RigidbodyInterpolation interpolation;
        /// <summary>
        /// Controls whether physics affects the rigidbody.
        /// </summary>
        public bool isKinematic;
        /// <summary>
        /// The mass of the rigidbody.
        /// </summary>
        public float mass;
        /// <summary>
        /// The maximimum angular velocity of the rigidbody measured in radians per second. (Default 7) range { 0, infinity }.
        /// </summary>
        public float maxAngularVelocity;
        /// <summary>
        /// Maximum velocity of a rigidbody when moving out of penetrating state.
        /// </summary>
        public float maxDepenetrationVelocity;
        /// <summary>
        /// The position of the rigidbody.
        /// </summary>
        public Vector3 position;
        /// <summary>
        /// The rotation of the Rigidbody.
        /// </summary>
        public Quaternion rotation;
        /// <summary>
        /// The mass-normalized energy threshold, below which objects start going to sleep.
        /// </summary>
        public float sleepThreshold;
        /// <summary>
        /// The solverIterations determines how accurately Rigidbody joints and collision contacts are resolved. Overrides <c>Physics.defaultSolverIterations</c>. Must be positive.
        /// </summary>
        public int solverIterations;
        /// <summary>
        /// The solverVelocityIterations affects how how accurately Rigidbody joints and collision contacts are resolved.
        /// Overrides <c>Physics.defaultSolverVelocityIterations</c>. Must be positive.
        /// </summary>
        public int solverVelocityIterations;
        /// <summary>
        /// Controls whether gravity affects this rigidbody.
        /// </summary>
        public bool useGravity;
        /// <summary>
        /// The velocity vector of the rigidbody. It represents the rate of change of Rigidbody position.
        /// </summary>
        public Vector3 velocity;
        /// <summary>
        /// The center of mass of the rigidbody in world space (Read Only).
        /// </summary>
        public readonly Vector3 worldCenterOfMass;
        /// <summary>
        /// Applies a force to a rigidbody that simulates explosion effects.
        /// </summary>
        /// <param name="explosionForce">The force of the explosion (which may be modified by distance).</param>
        /// <param name="explosionPosition">The centre of the sphere within which the explosion has its effect.</param>
        /// <param name="explosionRadius">The radius of the sphere within which the explosion has its effect.</param>
        /// <param name="upwardsModifier">Adjustment to the apparent position of the explosion to make it seem to lift objects.</param>
        /// <param name="mode">The method used to apply the force to its targets.</param>
        public void AddExplosionForce(float explosionForce, Vector3 explosionPosition, float explosionRadius, float upwardsModifier = 0.0f, ForceMode mode = ForceMode.Force);
        /// <summary>
        /// Adds a force to the <c>Rigidbody</c>.
        /// </summary>
        /// <param name="force">Force vector in world coordinates.</param>
        /// <param name="mode">Type of force to apply.</param>
        public void AddForce(Vector3 force, ForceMode mode = ForceMode.Force);
        /// <summary>
        /// Applies <c>force</c> at <c>position</c>. As a result this will apply a torque and force on the object.
        /// </summary>
        /// <param name="force">Force vector in world coordinates.</param>
        /// <param name="position">Position in world coordinates.</param>
        /// <param name="mode">Type of force to apply.</param>
        public void AddForceAtPosition(Vector3 force, Vector3 position, ForceMode mode = ForceMode.Force);
        /// <summary>
        /// Adds a force to the rigidbody relative to its coordinate system.
        /// </summary>
        /// <param name="force">Force vector in local coordinates.</param>
        /// <param name="mode">Type of force to apply.</param>
        public void AddRelativeForce(Vector3 force, ForceMode mode = ForceMode.Force);
        /// <summary>
        /// Adds a torque to the rigidbody relative to its coordinate system.
        /// </summary>
        /// <param name="torque">Torque vector in local coordinates.</param>
        /// <param name="mode">Type of force to apply.</param>
        public void AddRelativeTorque(Vector3 torque, ForceMode mode = ForceMode.Force);
        /// <summary>
        /// Adds a torque to the rigidbody.
        /// </summary>
        /// <param name="torque">Torque vector in world coordinates.</param>
        /// <param name="mode">Type of force to apply.</param>
        public void AddTorque(Vector3 torque, ForceMode mode = ForceMode.Force);
        /// <summary>
        /// The closest point to the bounding box of the attached colliders.
        /// </summary>
        public Vector3 ClosestPointOnBounds(Vector3 position);
        /// <summary>
        /// The velocity of the rigidbody at the point <c>worldPoint</c> in global space.
        /// </summary>
        public Vector3 GetPointVelocity(Vector3 worldPoint);
        /// <summary>
        /// The velocity relative to the rigidbody at the point <c>relativePoint</c>.
        /// </summary>
        public Vector3 GetRelativePointVelocity(Vector3 relativePoint);
        /// <summary>
        /// Is the rigidbody sleeping?
        /// </summary>
        public bool IsSleeping();
        /// <summary>
        /// Moves the kinematic Rigidbody towards <c>position</c>.
        /// </summary>
        /// <param name="position">Provides the new position for the Rigidbody object.</param>
        public void MovePosition(Vector3 position);
        /// <summary>
        /// Rotates the rigidbody to <c>rotation</c>.
        /// </summary>
        /// <param name="rotation">The new rotation for the Rigidbody.</param>
        public void MoveRotation(Quaternion rotation);
        /// <summary>
        /// Reset the center of mass of the rigidbody.
        /// </summary>
        public void ResetCenterOfMass();
        /// <summary>
        /// Reset the inertia tensor value and rotation.
        /// </summary>
        public void ResetInertiaTensor();
        /// <summary>
        /// Sets the mass based on the attached colliders assuming a constant density.
        /// </summary>
        public void SetDensity(float density);
        /// <summary>
        /// Forces a rigidbody to sleep at least one frame.
        /// </summary>
        public void Sleep();
        /// <summary>
        /// bool True when the rigidbody sweep intersects any collider, otherwise false.
        /// </summary>
        /// <param name="direction">The direction into which to sweep the rigidbody.</param>
        /// <param name="hitInfo">If true is returned, hitInfo will contain more information about where the collider was hit (See Also: RaycastHit).</param>
        /// <param name="maxDistance">The length of the sweep.</param>
        /// <param name="queryTriggerInteraction">Specifies whether this query should hit Triggers.</param>
        /// <returns></returns>
        public bool SweepTest(Vector3 direction, out RaycastHit hitInfo, float maxDistance = Mathf.Infinity, QueryTriggerInteraction queryTriggerInteraction = QueryTriggerInteraction.UseGlobal);
        /// <summary>
        /// RaycastHit[] An array of all colliders hit in the sweep.
        /// </summary>
        /// <param name="direction">The direction into which to sweep the rigidbody.</param>
        /// <param name="maxDistance">The length of the sweep.</param>
        /// <param name="queryTriggerInteraction">Specifies whether this query should hit Triggers.</param>
        /// <returns></returns>
        public RaycastHit[] SweepTestAll(Vector3 direction, float maxDistance = Mathf.Infinity, QueryTriggerInteraction queryTriggerInteraction = QueryTriggerInteraction.UseGlobal);
        /// <summary>
        /// Forces a rigidbody to wake up.
        /// </summary>
        public void WakeUp();
    }

    /// <summary>
    /// A sphere-shaped primitive collider.
    /// </summary>
    public class SphereCollider : Collider {
        /// <summary>
        /// The center of the sphere in the object's local space.
        /// </summary>
        public Vector3 center;
        /// <summary>
        /// The radius of the sphere measured in the object's local space.
        /// </summary>
        public float radius;
    }

    /// <summary>
    /// The interface to get time information from Unity.
    /// </summary>
    public class Time {
        /// <summary>
        /// Slows game playback time to allow screenshots to be saved between frames.
        /// </summary>
        public static float captureDeltaTime;
        /// <summary>
        /// The reciprocal of <c>Time.captureDeltaTime</c>.
        /// </summary>
        public static int captureFramerate;
        /// <summary>
        /// The completion time in seconds since the last frame (Read Only).
        /// </summary>
        public static readonly float deltaTime;
        /// <summary>
        /// The interval in seconds at which physics and other fixed frame rate updates (like MonoBehaviour's FixedUpdate) are performed.
        /// </summary>
        public static float fixedDeltaTime;
        /// <summary>
        /// The time the latest FixedUpdate has started (Read Only). This is the time in seconds since the start of the game.
        /// </summary>
        public static float fixedTime;
        /// <summary>
        /// The timeScale-independent interval in seconds from the last fixed frame to the current one (Read Only).
        /// </summary>
        public static readonly float fixedUnscaledDeltaTime;
        /// <summary>
        /// The TimeScale-independant time the latest <c>FixedUpdate</c> has started (Read Only). This is the time in seconds since the start of the game.
        /// </summary>
        public static readonly float fixedUnscaledTime;
        /// <summary>
        /// The total number of frames that have passed (Read Only).
        /// </summary>
        public static readonly int frameCount;
        /// <summary>
        /// Returns true if called inside a fixed time step callback (like MonoBehaviour's <c>FixedUpdate</c>), otherwise returns false.
        /// </summary>
        public static bool inFixedTimeStep;
        /// <summary>
        /// The maximum time a frame can take.
        /// Physics and other fixed frame rate updates (like MonoBehaviour's FixedUpdate) will be performed only for this duration of time per frame.
        /// </summary>
        public static float maximumDeltaTime;
        /// <summary>
        /// The maximum time a frame can spend on particle updates.
        /// If the frame takes longer than this, then updates are split into multiple smaller updates.
        /// </summary>
        public static float maximumParticleDeltaTime;
        /// <summary>
        /// The real time in seconds since the game started (Read Only).
        /// </summary>
        public static readonly float realtimeSinceStartup;
        /// <summary>
        /// A smoothed out Time.deltaTime (Read Only).
        /// </summary>
        public static readonly float smoothDeltaTime;
        /// <summary>
        /// The time at the beginning of this frame (Read Only). This is the time in seconds since the start of the game.
        /// </summary>
        public static readonly float time;
        /// <summary>
        /// The scale at which time passes. This can be used for slow motion effects.
        /// </summary>
        public static float timeScale;
        /// <summary>
        /// The time this frame has started (Read Only). This is the time in seconds since the last level has been loaded.
        /// </summary>
        public static readonly float timeSinceLevelLoad;
        /// <summary>
        /// The timeScale-independent interval in seconds from the last frame to the current one (Read Only).
        /// </summary>
        public static readonly float unscaledDeltaTime;
        /// <summary>
        /// The timeScale-independant time for this frame (Read Only). This is the time in seconds since the start of the game.
        /// </summary>
        public static readonly float unscaledTime;
    }

    /// <summary>
    /// Position, rotation and scale of an object.
    /// <para>
    /// Every object in a Scene has a Transform.
    /// It's used to store and manipulate the position, rotation and scale of the object.
    /// Every Transform can have a parent, which allows you to apply position, rotation and scale hierarchically.
    /// This is the hierarchy seen in the Hierarchy pane.
    /// </para>
    /// </summary>
    public class Transform : Component {
        /// <summary>
        /// The number of children the parent Transform has.
        /// </summary>
        public int childCount;
        /// <summary>
        /// The rotation as Euler angles in degrees.
        /// </summary>
        public Vector3 eulerAngles;
        /// <summary>
        /// Returns a normalized vector representing the blue axis of the transform in world space.
        /// </summary>
        public Vector3 forward;
        /// <summary>
        /// Has the transform changed since the last time the flag was set to 'false'?
        /// </summary>
        public bool hasChanged;
        /// <summary>
        /// The transform capacity of the transform's hierarchy data structure.
        /// </summary>
        public int hierarchyCapacity;
        /// <summary>
        /// The number of transforms in the transform's hierarchy data structure.
        /// </summary>
        public int hierarchyCount;
        /// <summary>
        /// The rotation as Euler angles in degrees relative to the parent transform's rotation.
        /// </summary>
        public Vector3 localEulerAngles;
        /// <summary>
        /// Position of the transform relative to the parent transform.
        /// </summary>
        public Vector3 localPosition;
        /// <summary>
        /// The rotation of the transform relative to the transform rotation of the parent.
        /// </summary>
        public Quaternion localRotation;
        /// <summary>
        /// The scale of the transform relative to the GameObjects parent.
        /// </summary>
        public Vector3 localScale;
        /// <summary>
        /// Matrix that transforms a point from local space into world space (Read Only).
        /// </summary>
        public readonly Matrix4x4 localToWorldMatrix;
        /// <summary>
        /// The global scale of the object (Read Only).
        /// </summary>
        public readonly Vector3 lossyScale;
        /// <summary>
        /// The parent of the transform.
        /// </summary>
        public Transform parent;
        /// <summary>
        /// The world space position of the Transform.
        /// </summary>
        public Vector3 position;
        /// <summary>
        /// The red axis of the transform in world space.
        /// </summary>
        public Vector3 right;
        /// <summary>
        /// Returns the topmost transform in the hierarchy.
        /// </summary>
        public Transform root;
        /// <summary>
        /// A Quaternion that stores the rotation of the Transform in world space.
        /// </summary>
        public Quaternion rotation;
        /// <summary>
        /// The green axis of the transform in world space.
        /// </summary>
        public Vector3 up;
        /// <summary>
        /// Matrix that transforms a point from world space into local space (Read Only).
        /// </summary>
        public readonly Matrix4x4 worldToLocalMatrix;


        /// <summary>
        /// Unparents all children.
        /// </summary>
        public void DetachChildren();
        /// <summary>
        /// <c>Transform</c> The returned child transform or null if no child is found.
        /// </summary>
        /// <param name="n">Name of child to be found.</param>
        /// <returns></returns>
        public Transform Find(string n);
        /// <summary>
        /// <c>Transform</c> Transform child by index.
        /// </summary>
        /// <param name="index">Index of the child transform to return. Must be smaller than Transform.childCount.</param>
        /// <returns></returns>
        public Transform GetChild(int index);
        /// <summary>
        /// Gets the sibling index.
        /// </summary>
        public int GetSiblingIndex();
        /// <summary>
        /// Transforms a <c>direction</c> from world space to local space. The opposite of Transform.TransformDirection.
        /// </summary>
        public Vector3 InverseTransformDirection(Vector3 direction);
        /// <summary>
        /// Transforms <c>position</c> from world space to local space.
        /// </summary>
        public Vector3 InverseTransformPoint(Vector3 position);
        /// <summary>
        /// Transforms a <c>vector</c> from world space to local space. The opposite of Transform.TransformVector.
        /// </summary>
        public Vector3 InverseTransformVector(Vector3 vector);
        /// <summary>
        /// Is this transform a child of <c>parent</c>?
        /// </summary>
        public bool IsChildOf(Transform parent);
        /// <summary>
        /// Rotates the transform so the forward vector points at /target/'s current position.
        /// </summary>
        /// <param name="target">Object to point towards.</param>
        /// <param name="worldUp">Vector specifying the upward direction.</param>
        public void LookAt(Transform target, Vector3 worldUp = Vector3.up);
        /// <summary>
        /// Rotates the object around the given axis by the number of degrees defined by the given angle.
        /// </summary>
        /// <param name="axis">The axis to apply rotation to.</param>
        /// <param name="angle">The degrees of rotation to apply.</param>
        /// <param name="relativeTo">Determines whether to rotate the GameObject either locally to the GameObject or relative to the Scene in world space.</param>
        public void Rotate(Vector3 axis, float angle, Space relativeTo = Space.Self);
        /// <summary>
        /// Rotates the transform about axis passing through point in world coordinates by angle degrees.
        /// </summary>
        public void RotateAround(Vector3 point, Vector3 axis, float angle);
        /// <summary>
        /// Move the transform to the start of the local transform list.
        /// </summary>
        public void SetAsFirstSibling();
        /// <summary>
        /// Move the transform to the end of the local transform list.
        /// </summary>
        public void SetAsLastSibling();
        /// <summary>
        /// Set the parent of the transform.
        /// </summary>
        /// <param name="parent">The parent Transform to use.</param>
        /// <param name="worldPositionStays">
        /// If true, the parent-relative position,
        /// scale and rotation are modified such that the object keeps the same world space position,
        /// rotation and scale as before.
        /// </param>
        public void SetParent(Transform parent, bool worldPositionStays = false);
        /// <summary>
        /// Sets the world space position and rotation of the Transform component.
        /// </summary>
        public void SetPositionAndRotation(Vector3 position, Quaternion rotation);
        /// <summary>
        /// Sets the sibling index.
        /// </summary>
        /// <param name="index">Index to set.</param>
        public void SetSiblingIndex(int index);
        /// <summary>
        /// Transforms <c>direction</c> from local space to world space.
        /// </summary>
        public Vector3 TransformDirection(Vector3 direction);
        /// <summary>
        /// Transforms <c>position</c> from local space to world space.
        /// </summary>
        public Vector3 TransformPoint(Vector3 position);
        /// <summary>
        /// Transforms <c>vector</c> from local space to world space.
        /// </summary>
        public Vector3 TransformVector(Vector3 vector);
        /// <summary>
        /// Moves the transform in the direction and distance of <c>translation</c>.
        /// </summary>
        public void Translate(Vector3 translation, Space relativeTo = Space.Self);
    }
}