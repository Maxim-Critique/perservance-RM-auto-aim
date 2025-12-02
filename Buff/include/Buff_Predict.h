// #include <queue>
// #include <cstdint>
// #include <Eigen/Dense>
// class BigNewtonGauss {
// public:
//     BigNewtonGauss();
//     ~BigNewtonGauss() = default;

//     void reset(); // ģʽ�л���ʱ����Ҫ����

//     // �����Ƿ���Ԥ��
//     int fit(double angle,double speed,time_point timestamp);
//     // �� timestamp ʱ��糵�ĽǶ�
//     double getAngle(time_point timestamp);

// private:
//     std::deque<time_point> fit_timestamps;
//     std::deque<double> fit_speeds;

//     int min_fit_samples = 10; // �����Կ�ʼ��ϵ�֡��
//     int max_fit_samples = 100; // �����������֡��

//     double a = 0.8;
//     double omega = 1.9;
//     double phase = 0; // ��λ
//     double b = 1.0;

//     int initialized = 0;
//     double direction = 0.0; // 1.0 or -1.0;

//     time_point last_time = 0;
//     double last_angle = 0;

//     double a_init;      // a�ĳ�ʼֵ
//     double omega_init;  // omega�ĳ�ʼֵ
//     double phase_init;  // phase�ĳ�ʼֵ
//     double b_init;      // b�ĳ�ʼֵ
//     double lambda;      // �ͷ�Ȩ��
//     double lambda_a;
// };

