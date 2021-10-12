#define CATCH_CONFIG_MAIN
#if __has_include(<catch2/catch.hpp>)
    #include <catch2/catch.hpp>
#else
    #include <catch.hpp>
#endif

#include <algorithm>

#include "processing/iir_filter.h"

SCENARIO("Electric field IIR filter", "[]")
{
    filter_ctx ctx={{0}};
    GIVEN("0 values as input and 0 as ctx")
    {
        THEN("should return only 0")
        {
            for(int i =0;i<1000;i++)
            {
                REQUIRE(0==filter(0, &ctx));
            }
        }
    }
    GIVEN("some step value")
    {
        THEN("should converge to given value")
        {
            for(int tries=0;tries<100;tries++)
            {
                int value = 16000*((double)rand()/RAND_MAX);
                for(int i =0;i<400;i++)
                {
                    filter(value, &ctx);
                }
                double tolerance=std::max(4.,0.001*value);
                REQUIRE(tolerance>fabs(0.8909*value-filter(value,&ctx)));
            }
        }
    }
}
