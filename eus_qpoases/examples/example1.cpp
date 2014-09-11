// qpOASES testing

// calculate time
#include <cstdio>
#include <sys/time.h>
#include <unistd.h>
#include <stdarg.h>

struct __mtimer__ {
  double start;
  char msg[100];
  bool is_sec_print;
  __mtimer__(const char* format, ...)
  __attribute__((format(printf, 2, 3)))
  {
    va_list args;
    va_start(args, format);
    vsnprintf(msg, sizeof(msg), format, args);
    va_end(args);

    start = sec();
  }
  ~__mtimer__() {
    fprintf(stderr, "[%s] => %.6f [s]\n", msg, sec() - start);
  }
  double sec() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
  }
  operator bool() { return false; }
};

#define mtimer(...) if(__mtimer__ __b__ = __mtimer__(__VA_ARGS__));else


// testing codes
#include <qpOASES.hpp>

int main( )
{
  USING_NAMESPACE_QPOASES
    {
      printf( "demo-qp1\n");
      /* Setup data of first QP. */
      real_t H[2*2] = {2,1,1,4};
      real_t A[2*1] = {1};
      real_t g[2] = {-8,-12};
      real_t ub[1*2] = {1e10, 1e10};
      real_t lb[1*2] = {-1e10, -1e10};
      real_t ubA[1*1] = {1e10};
      real_t lbA[1*1] = {-1e10};
      /* Setting up QProblem object. */
      QProblem example( 2,1 );
      Options options;
      options.printLevel = PL_NONE;
      example.setOptions( options );
      /* Solve first QP. */
      int nWSR = 10;
      getSimpleStatus(example.init( H,g,A,lb,ub,lbA,ubA, nWSR ), BT_FALSE);
      /* Get and print solution of second QP. */
      real_t xOpt[2];
      example.getPrimalSolution( xOpt );
      printf( "  xOpt = [ %f, %f ]; objVal = %f\n\n", 
              xOpt[0],xOpt[1],example.getObjVal() );
      //      example.printOptions();
    }
  {
      printf( "demo-qp2\n");
      /* Setup data of first QP. */
      real_t H[2*2] = {2,1,1,4};
      real_t A[2*2] = { 2.0, 1.0};
      real_t g[2] = {-8,-12};
      real_t ub[1*2] = {1e10, 1e10};
      real_t lb[1*2] = {-1e10, -1e10};
      real_t ubA[1*2] = {2.0, 2.0};
      real_t lbA[1*2] = {2.0,2.0};
      /* Setting up QProblem object. */
      QProblem example( 2,1 );
      Options options;
      options.printLevel = PL_NONE;
      example.setOptions( options );
      /* Solve first QP. */
      int nWSR = 10;
      getSimpleStatus(example.init( H,g,A,lb,ub,lbA,ubA, nWSR ), BT_FALSE);
      /* Get and print solution of second QP. */
      real_t xOpt[2];
      example.getPrimalSolution( xOpt );
      printf( "  xOpt = [ %f, %f ]; objVal = %f\n\n", 
              xOpt[0],xOpt[1],example.getObjVal() );
      //      example.printOptions();
  }
  {
    printf( "demo-qp3\n");
    /* Setup data of first QP. */
    real_t H[2*2] = {4,1,1,2};
    real_t A[2*1] = {1, 2};
    real_t g[2] = {-3, -4};
    real_t ub[1*2] = {1e10, 1e10};
    real_t lb[1*2] = {0,0};
    real_t ubA[1*1] = {1};
    real_t lbA[1*1] = {1};
    /* Setting up QProblem object. */
    QProblem example( 2,1 );
    Options options;
    options.printLevel = PL_NONE;
    example.setOptions( options );
    /* Solve first QP. */
    int nWSR = 10;
    example.init( H,g,A,lb,ub,lbA,ubA, nWSR );
    /* Get and print solution of second QP. */
    real_t xOpt[2];
    example.getPrimalSolution( xOpt );
    printf( "  xOpt = [ %f, %f ]; objVal = %f\n\n", 
            xOpt[0],xOpt[1],example.getObjVal() );
    //    example.printOptions();
  }

  {
    printf( "demo-qp3, use hotstart\n");
    real_t H[2*2] = {4,1,1,2};
    real_t A[2*1] = {1, 2};
    real_t g[2] = {-3, -4};
    real_t ub[1*2] = {1e10, 1e10};
    real_t lb[1*2] = {0,0};
    real_t ubA[1*1] = {1};
    real_t lbA[1*1] = {1};
    printf( "  init\n");
    mtimer("init x 10 time") {
      for (size_t i = 0; i < 10; i++) {
        QProblem example( 2,1 );
        Options options;
        options.printLevel = PL_NONE;
        example.setOptions( options );
        int nWSR = 10;
        example.init( H,g,A,lb,ub,lbA,ubA, nWSR );
        real_t xOpt[2];
        example.getPrimalSolution( xOpt );
        printf( "  xOpt = [ %f, %f ]; objVal = %f\n\n", 
                xOpt[0],xOpt[1],example.getObjVal() );
      }
    }
    printf( "  hotstart\n");
    QProblem example( 2,1 );
    Options options;
    options.printLevel = PL_NONE;
    example.setOptions( options );
    int nWSR = 10;
    example.init( H,g,A,lb,ub,lbA,ubA, nWSR );
    real_t xOpt[2];
    example.getPrimalSolution( xOpt );
    printf( "  xOpt = [ %f, %f ]; objVal = %f\n\n", 
            xOpt[0],xOpt[1],example.getObjVal() );
    mtimer("hotstart x 10 time") {
      for (size_t i = 0; i < 10; i++) {
        int nWSR2 = 10;
        example.hotstart(g,lb,ub,lbA,ubA, nWSR2 );
        example.getPrimalSolution( xOpt );
        printf( "  xOpt = [ %f, %f ]; objVal = %f\n\n", 
                xOpt[0],xOpt[1],example.getObjVal() );
      }
    }
  }

  {
    printf( "demo-qp3, use SQP init and hotstart\n");
    real_t H[2*2] = {4,1,1,2};
    real_t A[2*1] = {1, 2};
    real_t g[2] = {-3, -4};
    real_t ub[1*2] = {1e10, 1e10};
    real_t lb[1*2] = {0,0};
    real_t ubA[1*1] = {1};
    real_t lbA[1*1] = {1};

    SQProblem example( 2,1 );
    Options options;
    options.printLevel = PL_NONE;
    example.setOptions( options );
    mtimer("init x 10 time") {
      for (size_t i = 0; i < 10; i++) {
        int nWSR = 10;
        example.init( H,g,A,lb,ub,lbA,ubA, nWSR);
        real_t xOpt[2];
        example.getPrimalSolution( xOpt );
        printf( "  xOpt = [ %f, %f ]; objVal = %f\n\n",
                xOpt[0],xOpt[1],example.getObjVal() );
      }
    }
    mtimer("hotstart x 10 time") {
      for (size_t i = 0; i < 10; i++) {
        int nWSR = 10;
        example.hotstart( H,g,A,lb,ub,lbA,ubA, nWSR);
        real_t xOpt[2];
        example.getPrimalSolution( xOpt );
        printf( "  xOpt = [ %f, %f ]; objVal = %f\n\n",
                xOpt[0],xOpt[1],example.getObjVal() );
      }
    }
  }
  return 0;
}


/*
 *	end of file
 */
