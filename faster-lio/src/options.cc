// Faster-LIO 전역 옵션 및 설정 변수들
// 알고리즘 파라미터와 프로그램 상태 제어 변수 정의

#include "options.h"

namespace faster_lio::options {

// 칼만 필터 최대 반복 횟수
int NUM_MAX_ITERATIONS = 0;

// 평면 추정 임계값 (단위: 미터)
float ESTI_PLANE_THRESHOLD = 0.1;

// 프로그램 종료 플래그
bool FLAG_EXIT = false;

}  // namespace faster_lio