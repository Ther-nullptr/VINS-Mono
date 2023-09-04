  typedef typename NumTraits<Scalar>::Real RealScalar;
  template<typename MatrixType>
#define OWN_CHIP
#ifdef OWN_CHIP
  static Index unblocked(MatrixType& mat)
  {
    // std::cout << "input matrix:" << std::endl;
    // std::cout << mat << std::endl;
    auto in_chip = [](double a) {
      if (a == 0) {
          return 0.0f;
      }
      
      int exp;
      double man;
      man = std::frexp(a, &exp);
      
      if (exp > 128) {
          return (a > 0) ? static_cast<float>(127.0f * std::pow(2.0, 121)) : static_cast<float>(-127.0f * std::pow(2.0, 121));
      } else if (exp < -133) {
          return 0.0f;
      }

      if (exp >= -127) {
          man = man * std::pow(2.0, 7);
          int tmp;
          tmp = int(man);
          return static_cast<float>(tmp * std::pow(2.0, exp - 7));
      } else {
          int e = exp + 134;
          man = man * std::pow(2.0, e);
          int tmp;
          tmp = int(man);
          return static_cast<float>(tmp * std::pow(2.0, -134));
      }
    };

    using std::sqrt;
    eigen_assert(mat.rows()==mat.cols());
    const Index size = mat.rows();
    for(Index i = 0; i < size; ++i)
    {
      double tmp = sqrt(mat.coeff(i,i));
      mat.coeffRef(i,i) = in_chip(tmp);
      for (Index j = i + 1; j < size; j++)
      {
        double t = mat.coeff(j,i) / mat.coeff(i,i);
        mat.coeffRef(j,i) = in_chip(t);
      }
      for (Index j = i + 1; j < size; j++)
      {
        for (Index h = j; h < size; h++)
        {
          mat.coeffRef(h, j) -= mat.coeff(h, i) * mat.coeff(j, i);
        }
      }
    }
    // std::cout << "output matrix:" << std::endl;
    // std::cout << mat << std::endl;
    return -1;
  }
