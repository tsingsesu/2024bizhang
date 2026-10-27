#ifndef MAI_CONE_GEOMETRY_H
#define MAI_CONE_GEOMETRY_H

/* Image-space plausibility checks, not a metric 30-degree calibration.
   Keep short white bands, but require a group spread diagonally across the road.
   This cannot by itself distinguish every painted feature from a real cone. */
#define MAI_CONE_GROUP_MIN_POINTS 3
#define MAI_CONE_GROUP_SPAN_X 16
#define MAI_CONE_GROUP_SPAN_Y 8
#define MAI_CONE_GROUP_LINE_TOL 3
#define MAI_CONE_GROUP_MAX_POINTS 48

typedef struct
{
    unsigned char support;
    unsigned char span_x;
    unsigned char span_y;
    unsigned char error;
} Mai_Cone_Group;

static int Mai_Cone_Abs(int value)
{
    return value < 0 ? -value : value;
}

static unsigned char Mai_Cone_Group_Valid(const unsigned char *px,
                                         const unsigned char *py,
                                         unsigned char count,
                                         Mai_Cone_Group *result)
{
    unsigned char x[MAI_CONE_GROUP_MAX_POINTS], y[MAI_CONE_GROUP_MAX_POINTS];
    unsigned char i, j, a, b, n = 0;
    result->support = result->span_x = result->span_y = result->error = 0;
    if(count > MAI_CONE_GROUP_MAX_POINTS) count = MAI_CONE_GROUP_MAX_POINTS;

    /* Two nearby pieces of one white band are not two independent cones. */
    for(i = 0; i < count; i ++)
    {
        for(j = 0; j < n; j ++)
            if(Mai_Cone_Abs((int)px[i] - x[j]) <= 5 &&
               Mai_Cone_Abs((int)py[i] - y[j]) <= 3) break;
        if(j == n)
        {
            x[n] = px[i]; y[n] = py[i]; n ++;
        }
    }
    if(n < MAI_CONE_GROUP_MIN_POINTS)
    {
        result->support = n;
        return 0;
    }

    /* Fit candidate lines through pairs, allowing unrelated noise points. */
    for(a = 0; a < n; a ++)
        for(b = a + 1; b < n; b ++)
        {
            int dx = (int)x[b] - x[a], dy = (int)y[b] - y[a];
            int scale = Mai_Cone_Abs(dx) + Mai_Cone_Abs(dy);
            int error_sum = 0;
            unsigned char support = 0, min_x = 255, max_x = 0, min_y = 255, max_y = 0;
            if(Mai_Cone_Abs(dx) < MAI_CONE_GROUP_SPAN_X ||
               Mai_Cone_Abs(dy) < MAI_CONE_GROUP_SPAN_Y) continue;
            for(i = 0; i < n; i ++)
            {
                int cross = Mai_Cone_Abs(((int)x[i] - x[a]) * dy -
                                        ((int)y[i] - y[a]) * dx);
                if(cross > MAI_CONE_GROUP_LINE_TOL * scale) continue;
                support ++;
                error_sum += cross;
                if(x[i] < min_x) min_x = x[i];
                if(x[i] > max_x) max_x = x[i];
                if(y[i] < min_y) min_y = y[i];
                if(y[i] > max_y) max_y = y[i];
            }
            if(support > result->support)
            {
                result->support = support;
                result->span_x = max_x - min_x;
                result->span_y = max_y - min_y;
                result->error = (unsigned char)(error_sum / (support * scale));
            }
        }
    return (unsigned char)(result->support >= MAI_CONE_GROUP_MIN_POINTS &&
                           result->span_x >= MAI_CONE_GROUP_SPAN_X &&
                           result->span_y >= MAI_CONE_GROUP_SPAN_Y);
}

#endif
