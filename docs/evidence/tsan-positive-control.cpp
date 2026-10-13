// 证据：TSan 阳性对照的源码（故意造数据竞争）。
// 编译：g++ -fsanitize=thread -g -O1 -pthread tsan-positive-control.cpp -o tsan-race
// 运行：setarch $(uname -m) -R ./tsan-race        （加参数 fixed 则走加锁版本）

#include <cstdio>
#include <mutex>
#include <thread>

int race_var = 0;      // 故意：两个线程无同步地读写
int safe_var = 0;
std::mutex m;

void bump_racy() { for (int i = 0; i < 200000; ++i) ++race_var; }
void bump_safe() { for (int i = 0; i < 200000; ++i) { std::lock_guard<std::mutex> g(m); ++safe_var; } }

int main(int argc, char**) {
  // 参数 fixed 时走加锁版本（用来演示"修好之后 TSan 会安静"）
  bool fixed = argc > 1;
  std::thread a(fixed ? bump_safe : bump_racy);
  std::thread b(fixed ? bump_safe : bump_racy);
  a.join();
  b.join();
  std::printf("race_var=%d safe_var=%d\n", race_var, safe_var);
  return 0;
}
