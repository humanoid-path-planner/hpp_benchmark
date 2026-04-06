#include <map>
#include <vector>

namespace hpp {
namespace benchmark {

typedef double value_type;
typedef std::vector<value_type> result_t;
typedef std::map<std::string, result_t> results_t;

class BenchmarkRunner;

/** \brief Base class for all benchmarks.
 *  You should not inherit from this class directly but rather BenchmarkCase or
 *  BenchmarkNCase
 */
class BenchmarkBase {
 public:
  BenchmarkBase() : time_scale(1.) {}

  virtual ~BenchmarkBase() {}

  /** \brief Initialize the benchmark
   * Loading the robot, the environment, etc. takes place here.
   * \param Niter is the number of times the benchmark will be run.
   */
  virtual void setup(int Niter) = 0;

  /** \brief Deallocate what you allocated.
   */
  virtual void clean() = 0;

 protected:
  /** \brief Internal method
   * You should never call or reimplement this yourself, unless you know what
   * you do.
   */
  virtual void run(int N, const std::string& name) = 0;

  /// Scale the elapsed time by this amount.
  /// This is useful for bencharking more accurately fast benchmarks. In this
  /// one can implement \ref run such that it run N times the same thing,
  /// and set the \c time_scale to 1/N.
  value_type time_scale;

  friend class BenchmarkRunner;
};  // class BenchmarkBase

/** \brief Base class for benchmarks which have a single case.
 *
 *  This case will be iterated several times.
 */
class BenchmarkCase : public BenchmarkBase {
 public:
  virtual ~BenchmarkCase() {}

  /** \brief Initialize the problem
   * for the iteration \c iIter.
   */
  virtual void initializeProblem(int iIter) = 0;

  /** \brief Benchmarked function.
   * In this function goes the code whose execution time will be estimated.
   */
  virtual void solveProblem() = 0;

  /** \brief Save extra results
   * The execution time of solveProblem has already been stored in
   * "Time (unit)". You may add other result to \c results.
   */
  virtual void saveResolutionResult(results_t& results) = 0;

  /** \brief Tells whether the last run was successful.
   * At the moment, this is only informative.
   * \note By default, the solution is considered valid.
   */
  virtual bool validateSolution() { return true; }

 private:
  void run(int N, const std::string& name);
};  // class BenchmarkCase

/** \brief Base class for benchmarks which have a several cases.
 *
 *  This is useful when several benchmark share the same environment.
 *
 *  Each case will be iterated several times.
 */
class BenchmarkNCase : public BenchmarkBase {
 public:
  virtual ~BenchmarkNCase() {}

  /** \brief Comprehensive name of each case.
   */
  virtual std::vector<std::string> names() = 0;

  /** \brief Initialize the problem for a particular case.
   * for the iteration \c iIter.
   * \param iIter the repetition number,
   * \param iCase the benchmark case index in \ref names.
   */
  virtual void initializeProblem(int iIter, int iCase) = 0;

  /** \copydoc BenchmarkCase::solveProblem
   */
  virtual void solveProblem() = 0;

  /** \copydoc BenchmarkCase::saveResolutionResult
   */
  virtual void saveResolutionResult(results_t& results) = 0;

  /** \copydoc BenchmarkCase::validateSolution
   */
  virtual bool validateSolution() { return true; }

 private:
  void run(int N, const std::string& name);
};  // class BenchmarkNCase

/** \brief registration of a benchmark.
 * \note macro \c REGISTER ease the use of this function.
 */
void registerBenchmark(BenchmarkBase* p, const std::string& name);
}  // namespace benchmark
}  // namespace hpp

/** \brief Register a benchmark.
 */
#define REGISTER(CLASS, ID)                                  \
  struct __registerer__##ID {                                \
    __registerer__##ID() {                                   \
      ::hpp::benchmark::registerBenchmark(new CLASS(), #ID); \
    }                                                        \
  };                                                         \
  __registerer__##ID __registered_instance__##ID
