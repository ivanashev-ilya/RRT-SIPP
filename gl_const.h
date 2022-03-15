#ifndef GL_CONST_H
#define	GL_CONST_H

#define CN_PI_CONSTANT 3.14159265359
#define CN_SQRT_TWO    1.41421356237
#define CN_INFINITY    1e12
#define CN_EPSILON     1e-9

//default values
#define CN_DEFAULT_SIZE                     0.5
#define CN_DEFAULT_RSPEED                   1.0
#define CN_DEFAULT_MSPEED                   1.0
#define CN_DEFAULT_SHEADING                 0
#define CN_DEFAULT_GHEADING                 -1 //i.e. whatever
#define CN_DEFAULT_RESCHEDULING             CN_RE_NO
#define CN_DEFAULT_INITIALPRIORITIZATION    CN_IP_FIFO
#define CNS_DEFAULT_INITIALPRIORITIZATION   CNS_IP_FIFO
#define CN_DEFAULT_LOGLVL                   CN_LOGLVL_NORM
#define CNS_DEFAULT_LOGLVL                  CNS_LOGLVL_NORM
#define CN_DEFAULT_TIMELIMIT                CN_INFINITY
#define CNS_DEFAULT_TIMELIMIT               "'infinity' (no limit)"
#define CN_DEFAULT_CONNECTEDNESS            2
#define CN_DEFAULT_ALLOWANYANGLE            true
#define CNS_DEFAULT_ALLOWANYANGLE           "true"
#define CN_DEFAULT_PLANFORTURNS             false
#define CNS_DEFAULT_PLANFORTURNS            "false"
#define CN_DEFAULT_ADDITIONALWAIT           0
#define CN_DEFAULT_STARTSAFEINTERVAL        0
#define CN_DEFAULT_INFLATEINTERVALS          0

#define CNS_TAG_DEF_PARAMS          "defaultparameters"
#define CNS_TAG_ATTR_SIZE           "size"
#define CNS_TAG_ATTR_MOVESPEED      "movespeed"
#define CNS_TAG_ATTR_ROTATIONSPEED  "rotationspeed"
#define CNS_TAG_OBSTACLE            "obstacle"
#define CNS_TAG_ATTR_ID             "id"
#define CNS_TAG_ATTR_SX             "start.x"
#define CNS_TAG_ATTR_SY             "start.y"
#define CNS_TAG_ATTR_GX             "goal.x"
#define CNS_TAG_ATTR_GY             "goal.y"
#define CNS_TAG_ATTR_DURATION       "duration"

//XML tags
#define CNS_TAG_ROOT "root"

    #define CNS_TAG_MAP             "map"
        #define CNS_TAG_CELLSIZE    "cellsize"
        #define CNS_TAG_WIDTH       "width"
        #define CNS_TAG_HEIGHT      "height"
        #define CNS_TAG_STX         "startx"
        #define CNS_TAG_STY         "starty"
        #define CNS_TAG_FINX        "finishx"
        #define CNS_TAG_FINY        "finishy"
        #define CNS_TAG_GRID        "grid"
            #define CNS_TAG_ROW     "row"

    #define CNS_TAG_ALG             "algorithm"
        #define CNS_TAG_ST          "searchtype"
        #define CNS_TAG_HW          "hweight"
        #define CNS_TAG_MT          "metrictype"
        #define CNS_TAG_BT          "breakingties"
        #define CNS_TAG_AS          "allowsqueeze"
        #define CNS_TAG_AD          "allowdiagonal"
        #define CNS_TAG_CC          "cutcorners"
        #define CNS_TAG_TL          "timelimit"
        #define CNS_TAG_ME          "max_edge"
        #define CNS_TAG_SEED        "seed"

    #define CNS_TAG_OPT             "options"
        #define CNS_TAG_LOGLVL      "loglevel"
        #define CNS_TAG_LOGPATH     "logpath"
        #define CNS_TAG_LOGFN       "logfilename"

    #define CNS_TAG_DYNAMICOBSTACLES "dynamicobstacles"

    #define CNS_TAG_LOG             "log"
        #define CNS_TAG_MAPFN       "mapfilename"
        #define CNS_TAG_SUM         "summary"
        #define CNS_TAG_PATH        "path"
        #define CNS_TAG_LPLEVEL     "lplevel"
        #define CNS_TAG_AGENT       "agent"
        #define CNS_TAG_HPLEVEL     "hplevel"
            #define CNS_TAG_SECTION "section"
        #define CNS_TAG_LOWLEVEL    "lowlevel"
            #define CNS_TAG_STEP    "step"
            #define CNS_TAG_OPEN    "open"
            #define CNS_TAG_POINT   "node"
            #define CNS_TAG_CLOSE   "close"

//XML tags' attributes
    #define CNS_TAG_ATTR_NUMOFSTEPS     "numberofsteps"
    #define CNS_TAG_ATTR_NODESCREATED   "nodescreated"
    #define CNS_TAG_ATTR_LENGTH         "length"
    #define CNS_TAG_ATTR_LENGTH_SCALED  "length_scaled"
    #define CNS_TAG_ATTR_TIME           "time"
    #define CNS_TAG_ATTR_X              "x"
    #define CNS_TAG_ATTR_Y              "y"
    #define CNS_TAG_ATTR_NUM            "number"
    #define CNS_TAG_ATTR_F              "F"
    #define CNS_TAG_ATTR_G              "g"
    #define CNS_TAG_ATTR_PARX           "parent_x"
    #define CNS_TAG_ATTR_PARY           "parent_y"
    #define CNS_TAG_ATTR_STX            "start.x"
    #define CNS_TAG_ATTR_STY            "start.y"
    #define CNS_TAG_ATTR_FINX           "finish.x"
    #define CNS_TAG_ATTR_FINY           "finish.y"
    #define CNS_TAG_ATTR_GOALX          "goal.x"
    #define CNS_TAG_ATTR_GOALY          "goal.y"


//Search Parameters
    #define CN_SP_ST 0

        #define CNS_SP_ST_RRT           "rrt"

        #define CN_SP_ST_RRT            0

        #define CNS_SP_ST_RRT_SIPP      "rrt_sipp"

        #define CN_SP_ST_RRT_SIPP       1

        #define CNS_SP_ST_RRT_STAR_SIPP "rrt_star_sipp"

        #define CN_SP_ST_RRT_STAR_SIPP  2

    #define CN_SP_AD 1 //AllowDiagonal

    #define CN_SP_CC 2 //CutCorners

    #define CN_SP_AS 3 //AllowSqueeze

    #define CN_SP_MT 4 //MetricType

        #define CNS_SP_MT_DIAG  "diagonal"
        #define CNS_SP_MT_MANH  "manhattan"
        #define CNS_SP_MT_EUCL  "euclidean"
        #define CNS_SP_MT_CHEB  "chebyshev"

        #define CN_SP_MT_DIAG   0
        #define CN_SP_MT_MANH   1
        #define CN_SP_MT_EUCL   2
        #define CN_SP_MT_CHEB   3

    #define CN_SP_HW 5 //HeuristicWeight

    #define CN_SP_BT 6 //BreakingTies

        #define CNS_SP_BT_GMIN "g-min"
        #define CNS_SP_BT_GMAX "g-max"

        #define CN_SP_BT_GMIN 0
        #define CN_SP_BT_GMAX 1



    //Log Configuration
    #define CN_LP_LEVEL 0

        #define CN_LP_LEVEL_NOPE_VALUE      "0"
        #define CN_LP_LEVEL_NOPE_WORD       "none"
        #define CN_LP_LEVEL_TINY_VALUE      "0.5"
        #define CN_LP_LEVEL_TINY_WORD       "tiny"
        #define CN_LP_LEVEL_SHORT_VALUE     "1"
        #define CN_LP_LEVEL_SHORT_WORD      "short"
        #define CN_LP_LEVEL_MEDIUM_VALUE    "1.5"
        #define CN_LP_LEVEL_MEDIUM_WORD     "medium"
        #define CN_LP_LEVEL_FULL_VALUE      "2"
        #define CN_LP_LEVEL_FULL_WORD       "full"

    #define CN_LP_PATH 1
    #define CN_LP_NAME 2


//Grid Cell
    #define CN_GC_NOOBS 0
    #define CN_GC_OBS   1

//Other
    #define CNS_OTHER_PATHSELECTION     "*"
    #define CNS_OTHER_MATRIXSEPARATOR   ' '

#endif

