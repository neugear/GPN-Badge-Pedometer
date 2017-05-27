template <typename T>
struct Vector3 {
  T x, y, z;

  Vector3() {
    x = y = z = 0.0f;
  }

  Vector3(T n) {
    x = y = z = n;
  }

  Vector3(T a, T b, T c) {
    x = a;
    y = b;
    z = c;
  }

  Vector3(const Vector3 &v) {
    x = v.x;
    y = v.y;
    z = v.z;
  }

  ~Vector3() {

  }

  Vector3<T> operator + (const Vector3<T> &v) const {
    Vector3 result;

    result.x = x + v.x;
    result.y = y + v.y;
    result.z = z + v.z;

    return result;
  }

  Vector3<T> operator - (const Vector3<T> &v) const {
    Vector3<T> result;

    result.x = x - v.x;
    result.y = y - v.y;
    result.z = z - v.z;

    return result;
  }

  Vector3<T> operator / (float scalar) const {
    Vector3<T> result;

    result.x = x / scalar;
    result.y = y / scalar;
    result.z = z / scalar;

    return result;
  }

  T& operator [] (int i) {
    if ( i <= 0) {
      return x;
    }
    else if (i == 1) {
      return y;
    }
    else if (i >= 2) {
      return z;
    }
  }


};

typedef Vector3<float> Vector3f;

