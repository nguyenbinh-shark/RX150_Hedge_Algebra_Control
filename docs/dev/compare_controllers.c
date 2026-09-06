#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "rx150_fuzzy_controller/src/fuzzy/fuzzy_type1.h"
#include "rx150_hac_controller/src/hac/hac.h"

int main() {
    // Default HAC parameters based on rx150_hac_controller/src/hac_node.cpp
    float a = 1.0;
    float b = 0.5;
    float c = 767.73;
    
    printf("%-8s %-8s %-12s %-12s %-12s\n", "e", "ed", "fuzzy_out", "hac_out", "diff");
    printf("----------------------------------------------------------\n");
    
    for (float e = -0.3; e <= 0.31; e += 0.1) {
        for (float ed = -0.3; ed <= 0.31; ed += 0.1) {
            float f_out = fuzzy_type1_eval(e, ed)*800;
            float h_out = hac_eval(e, ed, a, b, c);
            float diff = f_out - h_out;
            printf("%-8.2f %-8.2f %-12.4f %-12.4f %-12.4f\n", e, ed, f_out, h_out, diff);
        }
    }
    
    return 0;
}
