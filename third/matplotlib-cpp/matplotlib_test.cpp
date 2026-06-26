#include "matplotlibcpp.h"

#include <gtest/gtest.h>

namespace plt = matplotlibcpp;

static void plot(void)
{
    plt::plot({1,3,2,4});
    plt::show();
}

TEST(MatplotlibTest, DrawPlot)
{
    plot();
}
