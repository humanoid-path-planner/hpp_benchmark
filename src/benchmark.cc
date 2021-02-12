#include "benchmark.hh"

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <chrono>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <hpp/util/timer.hh>

typedef std::map<std::string, hpp::benchmark::BenchmarkBase*> Problems_t;
Problems_t problems;

void print_header(std::ostream& os, const std::string& status)
{
  int size = 10;
  int l = static_cast<int>(status.size());
  int before = (size - l) / 2;
  int after  = size - l - before;
  os << '[';
  for (int i = 0; i < before; ++i) os << ' ';
  os << status;
  for (int i = 0; i < after ; ++i) os << ' ';
  os << ']' << ' ';
}

class NullStream : public std::ostream {
  class NullBuffer : public std::streambuf {
    public:
      int overflow( int c ) { return c; }
  } m_nb;
  public:
  NullStream() : std::ostream( &m_nb ) {}
};

bool dirExists(const char* d)
{
  struct stat info;

  if(stat(d, &info) != 0)
    return false;
  else if(!(info.st_mode & S_IFDIR))
    return false;
  return true;
}

struct Output {
  Output ()
    : out_ (&std::cout)
    , res_ (&nullStream_)
  {
    using std::chrono::system_clock;
    system_clock::time_point now = system_clock::now();
    std::time_t t_now = system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t_now), "%F");
    today = oss.str();
  }

  void print_header(const char* status)
  {
    ::print_header(o(), status);
  }

  void init_progress_bar (int N)
  {
    for (int i = 0; i < N; ++i) o() << '.';
    for (int i = 0; i < N; ++i) o() << '\b';
    o() << std::flush;
  }

  void pb_ok ()
  {
    o() << '|' << std::flush;
  }

  void pb_bad ()
  {
    o() << '-' << std::flush;
  }

  void setOutputDir(const char *dir)
  {
    if (!dirExists(dir)) {
      std::cerr << "Directory does not exists: " << dir << std::endl;
      return;
    }
    outputDir_ = dir;
  }

  bool setupResultFileFromBenchmarkName(const std::string& benchName)
  {
    if (outputDir_.empty()) return false;
    std::string filename = outputDir_ + '/' + benchName + ".csv";
    if (resFile_.is_open()) resFile_.close();
    resFile_.open(filename);
    res_ = &resFile_;
    return resFile_.good();
  }

  bool setupResultFileFromBenchmarkSubcaseName(const std::string& benchName,
      const std::string& caseName)
  {
    if (outputDir_.empty()) return false;
    std::string dir (outputDir_ + '/' + benchName);
    if (!dirExists(dir.c_str())) {
      if (mkdir(dir.c_str(), 0777) != 0) {
        std::cerr << "Could not create directory " << dir << std::endl;
        res_ = &nullStream_;
        return false;
      }
    }
    std::string filename = dir + '/' + caseName + ".csv";
    if (resFile_.is_open()) resFile_.close();
    resFile_.open(filename);
    res_ = &resFile_;
    return resFile_.good();
  }

  void writeResult(const hpp::benchmark::results_t& results)
  {
    std::vector< std::vector<hpp::benchmark::value_type> > csvContent;
    bool first = true;
    r() << "Label";
    for (auto& result : results) {
      if (first) csvContent.resize(result.second.size());
      r() << "; " << result.first;
      int i = 0;
      for (auto v : result.second) csvContent[i++].push_back(v);
      first = false;
    }
    r() << '\n';
    for (const auto& line : csvContent) {
      r() << today;
      for (auto v : line)
        r() << "; " << v;
      r() << '\n';
    }
    closeResultFile();
  }

  void closeResultFile()
  {
    resFile_.close();
  }

  std::ostream& o() { return *out_; };
  std::ostream& r() { return *res_; };

  std::string outputDir_;
  std::string today;

  std::ofstream outFile_;
  std::ofstream resFile_;
  NullStream nullStream_;

  std::ostream* out_;
  std::ostream* res_;
} output;

namespace hpp {
namespace benchmark {

void registerBenchmark(BenchmarkBase* p, const std::string& name)
{
  if (!problems.insert(std::make_pair(name, p)).second) {
    std::invalid_argument("Problem " + name + " already defined.");
    delete p;
  }
}

void stats (result_t values, value_type& mean, value_type& std_dev)
{
  mean = 0;
  std_dev = 0;
  int n = 0;
  for (auto& v : values) {
    if (!isnan(v) && !isinf(v)) {
      mean += v;
      std_dev += v*v;
      ++n;
    }
  }
  mean /= n;
  std_dev /= n;
  std_dev = std::sqrt(std_dev - mean*mean);
}

void BenchmarkCase::run (int N, const std::string& name)
{
  debug::Timer timer;

  output.print_header("RUN");
  output.o() << "Benchmark " << name << ' ' << std::flush;

  setup(N);

  results_t results;
  output.init_progress_bar(N);
  for (int i = 0; i < N; ++i) {
    initializeProblem(i);
    timer.start();
    solveProblem();
    timer.stop();
    results["Time (s)"].push_back(time_scale * static_cast<value_type>(timer.duration()));
    saveResolutionResult(results);
    if (validateSolution()) output.pb_ok();
    else                    output.pb_bad();
  }

  clean();
  output.o() << std::endl;

  if (output.setupResultFileFromBenchmarkName(name))
    output.writeResult(results);

  value_type mean, std_dev;
  for (auto& result : results) {
    output.print_header("Result");
    stats(result.second, mean, std_dev);
    output.o() << result.first << '\t' << mean << " +/- " << std_dev << '\n';
  }
}

void BenchmarkNCase::run (int N, const std::string& name)
{
  debug::Timer timer;

  output.print_header("SETUP");
  output.o() << "Benchmark " << name << ' ' << std::flush;
  setup(N);

  output.o() << '\n';
  std::vector<std::string> cases (names());
  int iCase = 0;
  for (const std::string& ncase : cases) {
    output.print_header("RUN");
    output.o() << "Benchmark " << name << '/' << ncase << ' ' << std::flush;

    results_t results;
    output.init_progress_bar(N);
    for (int i = 0; i < N; ++i) {
      initializeProblem(i, iCase);
      timer.start();
      solveProblem();
      timer.stop();
      results["Time (s)"].push_back(time_scale * static_cast<value_type>(timer.duration()));
      saveResolutionResult(results);
      if (validateSolution()) output.pb_ok();
      else                    output.pb_bad();
    }
    output.o() << std::endl;

    if (output.setupResultFileFromBenchmarkSubcaseName(name, ncase))
      output.writeResult(results);
    value_type mean, std_dev;
    for (auto& result : results) {
      stats(result.second, mean, std_dev);
      output.print_header("Result");
      output.o() << result.first << '\t' << mean << " +/- " << std_dev << '\n';
    }
    output.o() << std::flush;
    ++iCase;
  }

  clean();
}

/// \cond INTERNAL
/** \brief Internal class used to run the benchmarks
 */
class BenchmarkRunner {
public:
  static void run(int N, const std::string& name, hpp::benchmark::BenchmarkBase* problem)
  {
    problem->run(N, name);
  }
};
/// \endcond

} // namespace benchmark
} // namespace hpp

void usage(const char* name)
{
  std::cout << name << '\n'
    << " --help           show usage and exit\n"
    << " --list           list benchmarks and exit\n"
    << " --run <name>     run only one benchmark (instead of all by default). All options after this are ignored.\n"
    << " --output <dir>   directory where to write the benchmark results. If not specified, result are saved.\n"
    << " --label <label>  label (version, date...) indicating when the script was run (written in the result file).\n"
    << " -N <integer>     set the number of repetition for a benchmark\n"
    << std::flush;
}

#define CHECK_REMAINING_ARGC(i,n,msg) if (i + n >= argc) { std::cerr << "Expected " << n << " argument(s) after " << msg << '\n'; return 1; }
int main(int argc, char**argv)
{
  using hpp::benchmark::BenchmarkRunner;

  int N = 20;
  int iarg = 0;
  bool run_all = true;
  // Parse arguments
  while (++iarg < argc) {
    if (strcmp(argv[iarg], "--help") == 0) {
      usage(argv[0]);
      return 0;
    } else if (strcmp(argv[iarg], "--list") == 0) {
      std::cout << "Available benchmarks are:\n";
      for (auto& pair : problems)
        std::cout << pair.first << '\n';
      return 0;
    } else if (strcmp(argv[iarg], "--run") == 0) {
      CHECK_REMAINING_ARGC(iarg, 1, "--run");
      std::string name (argv[++iarg]);
      if (problems.count(name) == 0) {
        std::cerr << name << " not found.\n";
        return 1;
      }
      BenchmarkRunner::run(N, name, problems[name]);
      run_all = false;
    } else if (strcmp(argv[iarg], "--output") == 0) {
      CHECK_REMAINING_ARGC(iarg, 1, "--output");
      output.setOutputDir(argv[++iarg]);
    } else if (strcmp(argv[iarg], "--label") == 0) {
      CHECK_REMAINING_ARGC(iarg, 1, "--label");
      output.today = argv[++iarg];
    } else if (strcmp(argv[iarg], "-N") == 0) {
      CHECK_REMAINING_ARGC(iarg, 1, "-N");
      N = (int)strtol(argv[++iarg],NULL,0);
    }
  }

  // Run the benchmarks
  if (run_all)
    for (auto& pair : problems)
      BenchmarkRunner::run(N, pair.first, pair.second);
  return 0;
}
