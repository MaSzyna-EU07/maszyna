/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

//---------------------------------------------------------------------------
module;
#include <cmath>
#include <istream>
#include <ostream>

export module eu07.utilities.float3d;
import eu07.glm;

// Deklaracje klas
export class float3 {
public:
    float x, y, z;
    float3(void) : x(0), y(0), z(0) {};
    float3(float a, float b, float c) : x(a), y(b), z(c) {};
    float Length() const;
    float LengthSquared() const;

    operator glm::vec3() const {
        return glm::vec3(x, y, z);
    }
};

export class float4 {
public:
    float x, y, z, w;
    float4() : x(0), y(0), z(0), w(1) {};
    float4(float a, float b, float c, float d) : x(a), y(b), z(c), w(d) {};
    float inline LengthSquared() const {
        return x * x + y * y + z * z + w * w;
    };
    float inline Length() const {
        return sqrt(x * x + y * y + z * z + w * w);
    };
};

export struct float8 {
    float3 Point;
    float3 Normal;
    float tu, tv;
};

export class float4x4 {
public:
    float e[16];

    void deserialize_float32(std::istream&);
    void deserialize_float64(std::istream&);
    void serialize_float32(std::ostream&);
    float4x4(void) {
        Identity();
    };
    float4x4(const float f[16]) {
        for (int i = 0; i < 16; ++i)
            e[i] = f[i];
    };
    float * operator()(int i) {
        return &e[i << 2];
    }
    const float * readArray(void) const {
        return e;
    }
    void Identity() {
        for (int i = 0; i < 16; ++i)
            e[i] = 0;
        e[0] = e[5] = e[10] = e[15] = 1.0f;
    }
    const float *operator[](int i) const {
        return &e[i << 2];
    };
    void InitialRotate() {
        float f;
        for (int i = 0; i < 16; i += 4) {
            e[i] = -e[i];
            f = e[i + 1];
            e[i + 1] = e[i + 2];
            e[i + 2] = f;
        }
    };
    float4x4 &Rotation(float const angle, float3 const &axis);
    bool IdentityIs() {
        for (int i = 0; i < 16; ++i)
            if (e[i] != (i % 5 ? 0.0 : 1.0))
                return false;
        return true;
    }
    void Quaternion(float4 *q);
    float3 *TranslationGet() {
        return (float3 *)(e + 12);
    }
};

// Funkcje globalne - każda z osobnym export
export inline bool operator==(const float3 &v1, const float3 &v2);
export inline float3 &operator+=(float3 &v1, const float3 &v2);
export inline float3 operator-(const float3 &v);
export inline float3 operator-(const float3 &v1, const float3 &v2);
export inline float3 operator+(const float3 &v1, const float3 &v2);
export inline float3 operator*(float3 const &v, float const k);
export inline float3 operator/(float3 const &v, float const k);
export inline float3 SafeNormalize(const float3 &v);
export inline float3 CrossProduct(float3 const &v1, float3 const &v2);
export inline float DotProduct(float3 const &v1, float3 const &v2);
export inline float3 Interpolate(float3 const &First, float3 const &Second, float const Factor);
export inline float4 operator*(const float4 &q1, const float4 &q2);
export inline float4 operator-(const float4 &q);
export inline float4 operator-(const float4 &q1, const float4 &q2);
export inline float4 operator+(const float4 &v1, const float4 &v2);
export inline float4 operator/(const float4 &v, float const k);
export inline float4 Normalize(const float4 &v);
export inline float Dot(const float4 &q1, const float4 &q2);
export inline float4 &operator*=(float4 &v1, float const d);
export inline float4 Slerp(const float4 &q0, const float4 &q1, float t);
export inline float3 operator*(const float4x4 &m, const float3 &v);
export inline glm::vec3 operator*(const float4x4 &m, const glm::vec3 &v);
export inline float4x4 operator*(const float4x4 &m1, const float4x4 &m2);
export inline float Det2x2(float a, float b, float c, float d);
export inline float Det3x3(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3);
export inline float Det(const float4x4 &m);
export inline bool operator==(const float4x4& v1, const float4x4& v2);

// Definicje funkcji składowych
inline float float3::Length() const {
    return std::sqrt(LengthSquared());
};

inline float float3::LengthSquared() const {
    return x * x + y * y + z * z;
}

inline float4x4 &float4x4::Rotation(float const Angle, float3 const &Axis) {
    auto const c = std::cos(Angle);
    auto const s = std::sin(Angle);
    auto const omc = 1.f - c;
    auto const axis = SafeNormalize(Axis);
    auto const xs = axis.x * s;
    auto const ys = axis.y * s;
    auto const zs = axis.z * s;
    auto const xyomc = axis.x * axis.y * omc;
    auto const xzomc = axis.x * axis.z * omc;
    auto const yzomc = axis.y * axis.z * omc;
    e[0] = axis.x * axis.x * omc + c;
    e[1] = xyomc + zs;
    e[2] = xzomc - ys;
    e[3] = 0;
    e[4] = xyomc - zs;
    e[5] = axis.y * axis.y * omc + c;
    e[6] = yzomc + xs;
    e[7] = 0;
    e[8] = xzomc + ys;
    e[9] = yzomc - xs;
    e[10] = axis.z * axis.z * omc + c;
    e[11] = 0;
    e[12] = 0;
    e[13] = 0;
    e[14] = 0;
    e[15] = 1;
    return *this;
};

// Definicje funkcji globalnych
export inline bool operator==(const float3 &v1, const float3 &v2) {
    return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
}

export inline float3 &operator+=(float3 &v1, const float3 &v2) {
    v1.x += v2.x;
    v1.y += v2.y;
    v1.z += v2.z;
    return v1;
}

export inline float3 operator-(const float3 &v) {
    return float3(-v.x, -v.y, -v.z);
}

export inline float3 operator-(const float3 &v1, const float3 &v2) {
    return float3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

export inline float3 operator+(const float3 &v1, const float3 &v2) {
    return float3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

export inline float3 operator*(float3 const &v, float const k) {
    return float3(v.x * k, v.y * k, v.z * k);
}

export inline float3 operator/(float3 const &v, float const k) {
    return float3(v.x / k, v.y / k, v.z / k);
}

export inline float3 SafeNormalize(const float3 &v) {
    auto const l = v.Length();
    float3 retVal;
    if (l == 0)
        retVal.x = retVal.y = retVal.z = 0;
    else
        retVal = v / l;
    return retVal;
}

export inline float3 CrossProduct(float3 const &v1, float3 const &v2) {
    return float3(v1.y * v2.z - v1.z * v2.y,
                  v2.x * v1.z - v2.z * v1.x,
                  v1.x * v2.y - v1.y * v2.x);
}

export inline float DotProduct(float3 const &v1, float3 const &v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

export inline float3 Interpolate(float3 const &First, float3 const &Second, float const Factor) {
    return First * (1.0f - Factor) + Second * Factor;
}

export inline float4 operator*(const float4 &q1, const float4 &q2) {
    return float4(q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
                  q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z,
                  q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x,
                  q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z);
}

export inline float4 operator-(const float4 &q) {
    return float4(-q.x, -q.y, -q.z, q.w);
}

export inline float4 operator-(const float4 &q1, const float4 &q2) {
    return -q1 * q2;
}

export inline float4 operator+(const float4 &v1, const float4 &v2) {
    return float4(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z, v1.w + v2.w);
}

export inline float4 operator/(const float4 &v, float const k) {
    return float4(v.x / k, v.y / k, v.z / k, v.w / k);
}

export inline float4 Normalize(const float4 &v) {
    auto const lengthsquared = v.LengthSquared();
    if (lengthsquared == 1.0)
        return v;
    if (lengthsquared == 0.0)
        return float4();
    else
        return v / std::sqrt(lengthsquared);
}

export inline float Dot(const float4 &q1, const float4 &q2) {
    return q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
}

export inline float4 &operator*=(float4 &v1, float const d) {
    v1.x *= d;
    v1.y *= d;
    v1.z *= d;
    v1.w *= d;
    return v1;
}

export inline float4 Slerp(const float4 &q0, const float4 &q1, float t) {
    float cosOmega = Dot(q0, q1);
    float4 new_q1(q1);
    if (cosOmega < 0.0f) {
        new_q1.x = -new_q1.x;
        new_q1.y = -new_q1.y;
        new_q1.z = -new_q1.z;
        new_q1.w = -new_q1.w;
        cosOmega = -cosOmega;
    }
    float k0, k1;
    if (cosOmega > 0.9999f) {
        k0 = 1.0f - t;
        k1 = t;
    }
    else {
        auto const sinOmega = std::sqrt(1.0f - cosOmega * cosOmega);
        auto const omega = std::atan2(sinOmega, cosOmega);
        auto const oneOverSinOmega = 1.0f / sinOmega;
        k0 = sin((1.0f - t) * omega) * oneOverSinOmega;
        k1 = sin(t * omega) * oneOverSinOmega;
    }
    return float4(q0.x * k0 + new_q1.x * k1,
                  q0.y * k0 + new_q1.y * k1,
                  q0.z * k0 + new_q1.z * k1,
                  q0.w * k0 + new_q1.w * k1);
}

export inline float3 operator*(const float4x4 &m, const float3 &v) {
    return float3(v.x * m[0][0] + v.y * m[1][0] + v.z * m[2][0] + m[3][0],
                  v.x * m[0][1] + v.y * m[1][1] + v.z * m[2][1] + m[3][1],
                  v.x * m[0][2] + v.y * m[1][2] + v.z * m[2][2] + m[3][2]);
}

export inline glm::vec3 operator*(const float4x4 &m, const glm::vec3 &v) {
    return glm::vec3(
        v.x * m[0][0] + v.y * m[1][0] + v.z * m[2][0] + m[3][0],
        v.x * m[0][1] + v.y * m[1][1] + v.z * m[2][1] + m[3][1],
        v.x * m[0][2] + v.y * m[1][2] + v.z * m[2][2] + m[3][2]);
}

export inline bool operator==(const float4x4& v1, const float4x4& v2) {
    for (size_t i = 0; i < 16; i++) {
        if (v1.e[i] != v2.e[i])
            return false;
    }
    return true;
}

export inline float4x4 operator*(const float4x4 &m1, const float4x4 &m2) {
    float4x4 retVal;
    for (int x = 0; x < 4; ++x)
        for (int y = 0; y < 4; ++y) {
            retVal(x)[y] = 0;
            for (int i = 0; i < 4; ++i)
                retVal(x)[y] += m1[i][y] * m2[x][i];
        }
    return retVal;
}

export inline float Det2x2(float a, float b, float c, float d) {
    return a * d - b * c;
}

export inline float Det3x3(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3) {
    return +a1 * Det2x2(b2, b3, c2, c3) - b1 * Det2x2(a2, a3, c2, c3) + c1 * Det2x2(a2, a3, b2, b3);
}

export inline float Det(const float4x4 &m) {
    float a1 = m[0][0], a2 = m[1][0], a3 = m[2][0], a4 = m[3][0];
    float b1 = m[0][1], b2 = m[1][1], b3 = m[2][1], b4 = m[3][1];
    float c1 = m[0][2], c2 = m[1][2], c3 = m[2][2], c4 = m[3][2];
    float d1 = m[0][3], d2 = m[1][3], d3 = m[2][3], d4 = m[3][3];
    return +a1 * Det3x3(b2, b3, b4, c2, c3, c4, d2, d3, d4) -
           b1 * Det3x3(a2, a3, a4, c2, c3, c4, d2, d3, d4) +
           c1 * Det3x3(a2, a3, a4, b2, b3, b4, d2, d3, d4) -
           d1 * Det3x3(a2, a3, a4, b2, b3, b4, c2, c3, c4);
}