#include<stdio.h>
#include<SDL.h>
#include <math.h>
/*------------------------------------------constants----------------------------------------------------
*/
#define WIDTH 1920
#define HEIGHT 1080
#define ASCPECT_RATIO WIDTH/HEIGHT
#define FOV 70
#define FPS_LIMIT 15
/*--------------------------------------------Structs------------------------------------------------------
*/
typedef struct Vec3f
{
    float x, y, z;
} Vec3f;

typedef struct Triangle
{
    Vec3f vertex[3];
    SDL_Color col;
} Triangle;

typedef struct Mesh
{
    int numberOfTriangles;
    Triangle triangles[1024];
} Mesh;

typedef struct Matri4x4
{
    float m[4][4];
} Matri4x4;



/*---------------------------Function Declaration---------------------------------------
*/
void ErrorQuitHandler(char* message);
void InputHandler();
void ProcessHandler();
void DisplayHandler();
int EngineInitHandler();
int FrameRateHandler(unsigned int delta);
float Radianify(float degrees);


void PopulateCubeMeshHandler();

Vec3f Matri4x4MultiplicationHandler(Vec3f i, Matri4x4 m, int must_w);
Vec3f Normalise(Vec3f v);
Vec3f CrossProduct(Vec3f a, Vec3f b);
Vec3f GetLine(Vec3f a, Vec3f b);
Vec3f scaleVec3f(Vec3f v, Vec3f scale_factor);
Vec3f addVec3f(Vec3f a, Vec3f b);
Vec3f subVec3f(Vec3f a, Vec3f b);
Vec3f Multiply_vector_scalar(Vec3f v, float s);

Mesh ReadOBJ(const char fname[]);

float DotProduct(Vec3f a, Vec3f b);
float VecLength(Vec3f v);


void DrawTriangle(Triangle t);
void DrawFillTriangHandler(Triangle t);

Triangle TriangleTransformHandler(Triangle t, Matri4x4 m, int must_w);
Triangle TriangleTranslateHandler(Triangle t, Vec3f v);

Vec3f GetTriangleNormal(Triangle t);
void DrawMesh(Mesh m, Vec3f position , Matri4x4 Matrix_Array[], int Number_of_transform_matrices ,SDL_Color color);


Matri4x4 Matrix_Matrix_Multiplication(Matri4x4 m1, Matri4x4 m2);
Matri4x4 create_Unit_Matrix();
Matri4x4 Matrix_MakeRotationY(float fAngleRad);
Matri4x4 Matrix_MakeRotationZ(float fAngleRad);
Matri4x4 Matrix_MakeRotationX(float fAngleRad);
Matri4x4 Matrix_MakeTranslation(float x, float y, float z);
Matri4x4 Matrix_Point_at(Vec3f pos, Vec3f target, Vec3f up);
Matri4x4 Quick_Inverse_Matrix(Matri4x4 m);

void DisplayMatrix(Matri4x4 m);


/*-----------------------------------Global Variables-----------------------------------------------------
*/
SDL_Renderer* renderer = NULL;
SDL_Window* window = NULL;


float Z0;
float fNear = 0.1f;
float fFar = 1000.0f;
float fFov = 90.0f;
float fAspectratio = ((float)HEIGHT/(float)WIDTH);
float fFovRad;

unsigned int delta;

float fTheta = 0.0f;
float Yaw = 0.0f;
float Xaw = 0.0f;


Mesh cube1Mesh;
Mesh Rohancket;

Vec3f camerapos = (Vec3f){0.0f,0.0f,0.0f};
Vec3f LookDir = (Vec3f){0.0f,0.0f,0.0f};
Vec3f vUp = (Vec3f){0.0f,1.0f,0.0f};
Vec3f Targetv = (Vec3f){ 0.0f,0.0f,1.0f };

Matri4x4 matProj;
Matri4x4 matrotZ;
Matri4x4 matrotY;
Matri4x4 matrotX;

SDL_Color commonColors[6] =
{
    (SDL_Color){255,0,0,255},
    (SDL_Color){0,255,0,255},
    (SDL_Color){0,0,255,255},
    (SDL_Color){255,255,0,255},
    (SDL_Color){0,255,255,255},
    (SDL_Color){255,0,255,255},
};



Vec3f ViewPos = (Vec3f){0.0f,0.0f,9.0f};

/* ----------------------------------------Flags----------------------------------------------------
*/
short RUNNING = 0;

/*
*/
/*-----------------------------------------MAIN-----------------------------------------------------
*/

int main(int argc, char** argv)
{
    unsigned int frames = 0;
    unsigned int prevtime = 0;
    Rohancket = ReadOBJ("rohancket.obj");
    printf("\n[Rohancket created]: Number of Triangles: %d \n", Rohancket.numberOfTriangles);

    delta = SDL_GetTicks() + FPS_LIMIT;
    RUNNING = EngineInitHandler();
    while(RUNNING == 0)
    {
        FrameRateHandler(delta);
        prevtime = SDL_GetTicks();
        InputHandler();
        ProcessHandler();
        DisplayHandler();

        delta =  SDL_GetTicks() + FPS_LIMIT;
        frames ++;
        printf(" [%f] ", frames/(delta/1000.0f));
    }


    if(renderer != NULL)
    {
        SDL_DestroyRenderer(renderer);
    }
    if(window!=NULL)
    {
        SDL_DestroyWindow(window);
    }
    SDL_Quit();

    return 0;
}
/*
                                                                                    |
                                                                                    |
                                                                                    |
                                                                                    |
                                                                                    |
                                                                                    |
                                                                                    |
                                                                                    |
*///                                                                                |
//-----------------------------------------------------------------------------------

/*---------------------------------------Function Definitions------------------------------------------
*/



void ErrorQuitHandler(char* message)
{
    printf(message);
    if(renderer != NULL)
    {
        SDL_DestroyRenderer(renderer);
    }
    if(window!=NULL)
    {
        SDL_DestroyWindow(window);
    }
    SDL_Quit();
}


int FrameRateHandler(unsigned int d)
{
    unsigned int ticks = SDL_GetTicks();
    if(d < ticks)
        return;
    else if(d> ticks + FPS_LIMIT)
        SDL_Delay(FPS_LIMIT);
    else
        SDL_Delay(d - ticks);
}

//Matrix Manipulation Functions

Vec3f Matri4x4MultiplicationHandler(Vec3f i, Matri4x4 m, int must_w)
{
    Vec3f o;

    o.x = i.x * m.m[0][0] + i.y * m.m[1][0] + i.z * m.m[2][0] + m.m[3][0];
    o.y = i.x * m.m[0][1] + i.y * m.m[1][1] + i.z * m.m[2][1] + m.m[3][1];
    o.z = i.x * m.m[0][2] + i.y * m.m[1][2] + i.z * m.m[2][2] + m.m[3][2];



    float w = i.x * m.m[0][3] + i.y * m.m[1][3] + i.z * m.m[2][3] + m.m[3][3];

    if(w !=  0.0f && must_w == 1)
    {
        o.x /= w;
        o.y/=w;
        o.z/=w;
    }

    return o;

}

Matri4x4 Matrix_Matrix_Multiplication(Matri4x4 m1, Matri4x4 m2)
{
    Matri4x4 m;
    m = (Matri4x4){
        {
         {0,0,0,0},
         {0,0,0,0},
         {0,0,0,0},
         {0,0,0,0}
        }
          };
    //float val = 0.0f;
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            for(int x = 0; x < 4; x++)
            {
                m.m[i][j] += (m1.m[i][x] * m2.m[x][j]);
            }
        }

    }
    return m;
}


Matri4x4 create_Unit_Matrix()
{
        Matri4x4 matrix;
		matrix.m[0][0] = 1.0f;
		matrix.m[1][1] = 1.0f;
		matrix.m[2][2] = 1.0f;
		matrix.m[3][3] = 1.0f;
		return matrix;
}

Matri4x4 Matrix_MakeRotationY(float fAngleRad)
{
		Matri4x4 matrix;
		matrix.m[0][0] = cosf(fAngleRad);
		matrix.m[0][2] = sinf(fAngleRad);
		matrix.m[2][0] = -sinf(fAngleRad);
		matrix.m[1][1] = 1.0f;
		matrix.m[2][2] = cosf(fAngleRad);
		matrix.m[3][3] = 1.0f;
		return matrix;
	}

Matri4x4 Matrix_MakeRotationZ(float fAngleRad)
{
    Matri4x4 matrix;
    matrix.m[0][0] = cosf(fAngleRad);
    matrix.m[0][1] = sinf(fAngleRad);
    matrix.m[1][0] = -sinf(fAngleRad);
    matrix.m[1][1] = cosf(fAngleRad);
    matrix.m[2][2] = 1.0f;
    matrix.m[3][3] = 1.0f;
    return matrix;
}


Matri4x4 Matrix_MakeRotationX(float fAngleRad)
	{
		Matri4x4 matrix;
		matrix.m[0][0] = 1.0f;
		matrix.m[1][1] = cosf(fAngleRad);
		matrix.m[1][2] = sinf(fAngleRad);
		matrix.m[2][1] = -sinf(fAngleRad);
		matrix.m[2][2] = cosf(fAngleRad);
		matrix.m[3][3] = 1.0f;
		return matrix;
	}


Matri4x4 Matrix_MakeTranslation(float x, float y, float z)
{
    Matri4x4 matrix;
    matrix.m[0][0] = 1.0f;
    matrix.m[1][1] = 1.0f;
    matrix.m[2][2] = 1.0f;
    matrix.m[3][3] = 1.0f;
    matrix.m[3][0] = x;
    matrix.m[3][1] = y;
    matrix.m[3][2] = z;
    return matrix;
}




void DisplayMatrix(Matri4x4 m)
{
    for(int i = 0; i < 4; i++)
    {
        for(int x = 0; x < 4; x++)
        {
            printf("%f , ",m.m[i][x]);
        }
        printf("\n");
    }
}
// Vector Utility functions
float VecLength(Vec3f v)
{
    return (sqrt(pow(v.x,2) + pow(v.y,2) + pow(v.z,2)));
}

Vec3f CrossProduct(Vec3f a, Vec3f b)
{
    return (Vec3f){(a.y * b.z)-(a.z * b.y), (a.z * b.x)-(a.x * b.z),(a.x * b.y)-(a.y * b.x)};
}

Vec3f Normalise(Vec3f v)
{
    float l =  sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    return (Vec3f){v.x/l, v.y/l, v.z/l};
}

float DotProduct(Vec3f a, Vec3f b)
{
    return (a.x*b.x + a.y*b.y + a.z*b.z);
}

Vec3f addVec3f(Vec3f a, Vec3f b)
{
    return (Vec3f){a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3f subVec3f(Vec3f a, Vec3f b)
{
    return (Vec3f){a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3f scaleVec3f(Vec3f v, Vec3f scale_factor)
{
    return (Vec3f){
        v.x * scale_factor.x,
        v.y * scale_factor.y,
        v.z * scale_factor.z};
}

Vec3f Multiply_vector_scalar(Vec3f v, float s)
{
    return (Vec3f){v.x * s, v.y * s, v.z * s};
}

Vec3f GetLine(Vec3f a, Vec3f b)
{
    Vec3f line;

    line.x = a.x - b.x;
    line.y = a.y - b.y;
    line.z = a.z - b.z;


    return line;
}

// Math
float Radianify(float degrees)
{
    return ((degrees/180)*M_PI);
}


// Engine Utility Functions;

void DrawFillTriangHandler(Triangle t)
{ //RohanTorul :P
    Vec3f O, A, B, AB, P;
    float length;
    O = t.vertex[0];
    A = t.vertex[1];
    B = t.vertex[2];


    AB = (Vec3f){(B.x - A.x),(B.y - A.y),0};

    length = VecLength(AB);

    AB = (Vec3f){AB.x/length, AB.y/length,0};

    for(float alpha = 0.0f; alpha <= length; alpha += 0.1f)
    {
        P = (Vec3f) {(A.x + (alpha * AB.x)), (A.y + (alpha * AB.y)) , 0};
        SDL_RenderDrawLineF(renderer,O.x,O.y,P.x,P.y);
    }
    return;
}


void DrawTriangle(Triangle t)
{
     SDL_RenderDrawLineF(renderer,
                                t.vertex[0].x,
                                t.vertex[0].y,
                                t.vertex[1].x,
                                t.vertex[1].y);
            SDL_RenderDrawLineF(renderer,
                                t.vertex[1].x,
                                t.vertex[1].y,
                                t.vertex[2].x,
                                t.vertex[2].y);
            SDL_RenderDrawLineF(renderer,
                                t.vertex[2].x,
                                t.vertex[2].y,
                                t.vertex[0].x,
                                t.vertex[0].y);
    return ;
}

Triangle TriangleTransformHandler(Triangle t, Matri4x4 m, int must_w)
{
    Triangle tri;

    tri.vertex[0] = Matri4x4MultiplicationHandler(t.vertex[0],m,must_w);
    tri.vertex[1] = Matri4x4MultiplicationHandler(t.vertex[1],m, must_w);
    tri.vertex[2] = Matri4x4MultiplicationHandler(t.vertex[2],m, must_w);

    return tri;
}

Triangle TriangleTranslateHandler(Triangle t, Vec3f v)
{
    Triangle tri;

    tri.vertex[0] = addVec3f(t.vertex[0],v);
    tri.vertex[1] = addVec3f(t.vertex[1],v);
    tri.vertex[2] = addVec3f(t.vertex[2],v);

    return tri;
}



int EngineInitHandler()
{
    if(SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        ErrorQuitHandler("Error when Initialising SDL");
        return 1;
    }
    printf("Initialised SDL");

    window = SDL_CreateWindow("Testing",SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,WIDTH,HEIGHT,0);
    if(window == NULL)
    {
        ErrorQuitHandler("Error in creating window");
        return 2;
    }
    printf("Window created");

    renderer = SDL_CreateRenderer(window,-1,SDL_RENDERER_SOFTWARE);
    if(renderer == NULL)
    {
        ErrorQuitHandler("Error in Creating renderer");
        return 3;
    }
    printf("Renderer created");

    /*
    printf("Initiating a mesh cube");
    PopulateCubeMeshHandler();
    printf("\n");
    for(int i = 0; i < cube1Mesh.numberOfTriangles; i++)
    {
        printf("( %f ,",cube1Mesh.triangles[i].vertex[0].x);
        printf(" %f, ",cube1Mesh.triangles[i].vertex[0].y);
        printf(" %f )",cube1Mesh.triangles[i].vertex[0].z);
        printf("\n");
    }
    printf("\n");
    printf("Mesh created");
    */
    /*
    printf("Calculating z0");
    Z0 = (WIDTH/2)/(tan(Radianify(FOV/2)));
    printf("Z0: %f", Z0);
    */

    fFovRad = 1.0f / tan(Radianify(fFov/2));
    printf("fovrad: %f", fFovRad);
    printf("aspect ratio: %f", fAspectratio);
    printf("\n");
    for(int i=0; i < 4; i ++)
    {
        for(int x=0; x < 4; x ++)
        {
            matProj.m[i][x] = 0;
            printf("%f ",matProj.m[i][x]);
        }
        printf("\n");
    }
    matProj.m[0][0]=fAspectratio * fFovRad;
    matProj.m[1][1]=fFovRad;
    matProj.m[2][2]=fFar/(fFar - fNear);
    matProj.m[3][2]=(-fFar * fNear)/(fFar - fNear);
    matProj.m[2][3]=1.0f;
    matProj.m[3][3]=0.0f;
    printf("\n");
    for(int i=0; i < 4; i ++)
    {
        for(int x=0; x < 4; x ++)
        {
            printf("%f ",matProj.m[i][x]);
        }
        printf("\n");
    }

    return 0;


}


void PopulateCubeMeshHandler()
{
    cube1Mesh.numberOfTriangles = 12;

    //SQUARE 1
    cube1Mesh.triangles[0].vertex[0]= (Vec3f)
    {
        0.0f,0.0f,0.0f
    };
    cube1Mesh.triangles[0].vertex[1]= (Vec3f)
    {
        0.0f,1.0f,0.0f
    };
    cube1Mesh.triangles[0].vertex[2]= (Vec3f)
    {
        1.0f,1.0f,0.0f
    };

    cube1Mesh.triangles[1].vertex[0]= (Vec3f)
    {
        0.0f,0.0f,0.0f
    };
    cube1Mesh.triangles[1].vertex[1]= (Vec3f)
    {
        1.0f,1.0f,0.0f
    };
    cube1Mesh.triangles[1].vertex[2]= (Vec3f)
    {
        1.0f,0.0f,0.0f
    };

    //SQUARE 2
    cube1Mesh.triangles[2].vertex[0]= (Vec3f)
    {
        1.0f,0.0f,0.0f
    };
    cube1Mesh.triangles[2].vertex[1]= (Vec3f)
    {
        1.0f,1.0f,0.0f
    };
    cube1Mesh.triangles[2].vertex[2]= (Vec3f)
    {
        1.0f,1.0f,1.0f
    };

    cube1Mesh.triangles[3].vertex[0]= (Vec3f)
    {
        1.0f,0.0f,0.0f
    };
    cube1Mesh.triangles[3].vertex[1]= (Vec3f)
    {
        1.0f,1.0f,1.0f
    };
    cube1Mesh.triangles[3].vertex[2]= (Vec3f)
    {
        1.0f,0.0f,1.0f
    };

    //SQUARE 3
    cube1Mesh.triangles[4].vertex[0]= (Vec3f)
    {
        1.0f,0.0f,1.0f
    };
    cube1Mesh.triangles[4].vertex[1]= (Vec3f)
    {
        1.0f,1.0f,1.0f
    };
    cube1Mesh.triangles[4].vertex[2]= (Vec3f)
    {
        0.0f,1.0f,1.0f
    };

    cube1Mesh.triangles[5].vertex[0]= (Vec3f)
    {
        1.0f,0.0f,1.0f
    };
    cube1Mesh.triangles[5].vertex[1]= (Vec3f)
    {
        0.0f,1.0f,1.0f
    };
    cube1Mesh.triangles[5].vertex[2]= (Vec3f)
    {
        0.0f,0.0f,1.0f
    };

    //SQUARE 4
    cube1Mesh.triangles[6].vertex[0]= (Vec3f)
    {
        0.0f,0.0f,1.0f
    };
    cube1Mesh.triangles[6].vertex[1]= (Vec3f)
    {
        0.0f,1.0f,1.0f
    };
    cube1Mesh.triangles[6].vertex[2]= (Vec3f)
    {
        0.0f,1.0f,0.0f
    };

    cube1Mesh.triangles[7].vertex[0]= (Vec3f)
    {
        0.0f,0.0f,1.0f
    };
    cube1Mesh.triangles[7].vertex[1]= (Vec3f)
    {
        0.0f,1.0f,0.0f
    };
    cube1Mesh.triangles[7].vertex[2]= (Vec3f)
    {
        0.0f,0.0f,0.0f
    };

    //SQUARE 5
    cube1Mesh.triangles[8].vertex[0]= (Vec3f)
    {
        0.0f,1.0f,0.0f
    };
    cube1Mesh.triangles[8].vertex[1]= (Vec3f)
    {
        0.0f,1.0f,1.0f
    };
    cube1Mesh.triangles[8].vertex[2]= (Vec3f)
    {
        1.0f,1.0f,1.0f
    };

    cube1Mesh.triangles[9].vertex[0]= (Vec3f)
    {
        0.0f,1.0f,0.0f
    };
    cube1Mesh.triangles[9].vertex[1]= (Vec3f)
    {
        1.0f,1.0f,1.0f
    };
    cube1Mesh.triangles[9].vertex[2]= (Vec3f)
    {
        1.0f,1.0f,0.0f
    };

    //SQUARE 6
    cube1Mesh.triangles[10].vertex[0]= (Vec3f)
    {
        1.0f,0.0f,1.0f
    };
    cube1Mesh.triangles[10].vertex[1]= (Vec3f)
    {
        0.0f,0.0f,1.0f
    };
    cube1Mesh.triangles[10].vertex[2]= (Vec3f)
    {
        0.0f,0.0f,0.0f
    };

    cube1Mesh.triangles[11].vertex[0]= (Vec3f)
    {
        1.0f,0.0f,1.0f
    };
    cube1Mesh.triangles[11].vertex[1]= (Vec3f)
    {
        0.0f,0.0f,0.0f
    };
    cube1Mesh.triangles[11].vertex[2]= (Vec3f)
    {
        1.0f,0.0f,0.0f
    };

    //return *cube;
}


Mesh ReadOBJ(const char fname[])
{
    printf("called ReadOBJ \n");
    Mesh tempMesh;
    printf("temporary Mesh created \n");
    tempMesh.numberOfTriangles = 0;
    printf("attempting to open file \n");
    FILE* f = fopen(fname, "r");
    printf("[OPENED FILE].\n");
    char line[128];

    Vec3f pointsarray[1024]; int point_count = 0;



    while(fgets(line,128,f)!= NULL)
    {
        printf(" \n[Reading Line: %s] \n", line);
        switch(line[0])
        {
        case EOF:
            break;
        case 'v':
            printf("\n [VERTEX FOUND] \n");
            float px = 0.0f;
            float py = 0.0f;
            float pz = 0.0f;
             printf("Line: %s \n", line);
            sscanf(line,"%*c %f %f %f",&px, &py, &pz);
            printf("Point[%d]: %f %f %f \n",point_count,px, py, pz);
            pointsarray[point_count].x = px;
            pointsarray[point_count].y = py;
            pointsarray[point_count].z = pz;
            point_count++;
            break;
        case 'f':
            tempMesh.numberOfTriangles ++;
            printf("\n TRIANGLE %d FOUND \n", tempMesh.numberOfTriangles);
            int tn = tempMesh.numberOfTriangles-1;
            int p1 = 0;
            int p2 = 0;
            int p3 = 0;
            sscanf(line,"%*c %d %d %d",&p1, &p2, &p3 );
            printf("Indices: %d %d %d", p1, p2 ,p3);
            tempMesh.triangles[tn].vertex[0] = pointsarray[p1-1];
            tempMesh.triangles[tn].vertex[1] = pointsarray[p2-1];
            tempMesh.triangles[tn].vertex[2] = pointsarray[p3-1];
            break;
        default:
            break;
        }
    }

    fclose(f);
    printf("[FILE CLOSED.]\n");
    printf("[FILE READ.]\n");
    return tempMesh;
}

Vec3f GetTriangleNormal(Triangle t)
{
      // get normal
        Vec3f line1, line2, normal;
        line1 = GetLine(t.vertex[1],t.vertex[0]);

        line2 = GetLine(t.vertex[2], t.vertex[0]);

        normal = CrossProduct(line1,line2);

        return normal;
}

void DrawMesh(Mesh m, Vec3f position , Matri4x4 Matrix_Array[], int Number_of_transform_matrices ,SDL_Color color)
{
    for(int tn = 0; tn < m.numberOfTriangles; tn ++)
    {
        Triangle current_triangle = m.triangles[tn];
        Triangle triangle_transformed;
        Matri4x4 trans_Matrix;
        trans_Matrix = Matrix_Array[0];
        //apply transformations like rotation
        for(int mn = 1; mn < Number_of_transform_matrices; mn++)
        {
            trans_Matrix = Matrix_Matrix_Multiplication(trans_Matrix,Matrix_Array[mn]);
        }
        //apply translation to position
        triangle_transformed = TriangleTranslateHandler(current_triangle,position);

        Vec3f normal = Normalise(GetTriangleNormal(triangle_transformed));
         Vec3f CameraRay = subVec3f(triangle_transformed.vertex[0],camerapos);
        //check if the triangle is visible
        if(DotProduct(normal, CameraRay) < 0)
        {

            //illumination
             Vec3f light_direction = Normalise((Vec3f){0.0f, 0.0f, -1.0f});
              float cd = DotProduct(normal,light_direction);
             SDL_SetRenderDrawColor(renderer,(color.r *(cd)),(color.g * (cd)),(color.b * (cd)),color.a );

            //apply the projection matrix - Maps 3d onto 2d
            Triangle triangle_projected_2D = TriangleTransformHandler(triangle_projected_2D,matProj,1);

            // This transformation transformed all 3d points into 2d coords between(-1.0,-1.0) to (1.0,1.0)
            // before scaling, we have to make all points non zero
            // Since value are between -1 and 1, make them from 0 to 2
            triangle_projected_2D.vertex[0] = addVec3f(triangle_projected_2D.vertex[0], (Vec3f){1.0f,1.0f,0.0f});
            triangle_projected_2D.vertex[1] = addVec3f(triangle_projected_2D.vertex[1], (Vec3f){1.0f,1.0f,0.0f});
            triangle_projected_2D.vertex[2] = addVec3f(triangle_projected_2D.vertex[2], (Vec3f){1.0f,1.0f,0.0f});

             // Then scale them
            Vec3f scale_factor = (Vec3f){0.5f * WIDTH, 0.5f * HEIGHT, 1.0f};
            triangle_projected_2D.vertex[0] = scaleVec3f(triangle_projected_2D.vertex[0],scale_factor);
            triangle_projected_2D.vertex[1] = scaleVec3f(triangle_projected_2D.vertex[1],scale_factor);
            triangle_projected_2D.vertex[2] = scaleVec3f(triangle_projected_2D.vertex[2],scale_factor);



            //Draw the triangle

             //SDL_SetRenderDrawColor(renderer,150 * cd,150 * cd,150 * cd,255);
             //DrawFillTriangHandler(triprojected);
             //SDL_SetRenderDrawColor(renderer,0,0,0,255);
             //DrawTriangle(triprojected);


        }


    }
}


Matri4x4 Matrix_Point_at(Vec3f pos, Vec3f target, Vec3f up)
{

    //create new forward vector
    Vec3f newForward = subVec3f(target, pos);
    newForward = Normalise(newForward);

    //create new up vector
    Vec3f a = Multiply_vector_scalar(newForward,DotProduct(up, newForward));
    Vec3f newUp = subVec3f(up, a);
    newUp = Normalise(newUp);

    Vec3f newRight = CrossProduct(newUp, newForward);

    Matri4x4 pointAtMatrix;
    pointAtMatrix.m[0][0] =newRight.x;
    pointAtMatrix.m[0][1] =newRight.y;
    pointAtMatrix.m[0][2] =newRight.z;
    pointAtMatrix.m[0][3] =0.0f;
    pointAtMatrix.m[1][0] =newUp.x;
    pointAtMatrix.m[1][1] =newUp.y;
    pointAtMatrix.m[1][2] =newUp.z;
    pointAtMatrix.m[1][3] =0.0f;
    pointAtMatrix.m[2][0] =newForward.x;
    pointAtMatrix.m[2][1] =newForward.y;
    pointAtMatrix.m[2][2] =newForward.z;
    pointAtMatrix.m[2][3] =0.0f;
    pointAtMatrix.m[3][0] =pos.x;
    pointAtMatrix.m[3][1] =pos.y;
    pointAtMatrix.m[3][2] =pos.z;
    pointAtMatrix.m[3][3] =1.0f;

    return pointAtMatrix;

}

Matri4x4 Quick_Inverse_Matrix(Matri4x4 m)
{
    Matri4x4 inv_Matrix;
    Vec3f T = (Vec3f){m.m[3][0],m.m[3][1],m.m[3][2]};
    Vec3f A = (Vec3f){m.m[0][0],m.m[0][1],m.m[0][2]};
    Vec3f B = (Vec3f){m.m[1][0],m.m[1][1],m.m[1][2]};
    Vec3f C = (Vec3f){m.m[2][0],m.m[2][1],m.m[2][2]};

    inv_Matrix.m[0][0] =m.m[0][0] ;inv_Matrix.m[0][1] =m.m[1][0] ;inv_Matrix.m[0][2] =m.m[2][0] ;inv_Matrix.m[0][3]=0.0f;
    inv_Matrix.m[1][0] =m.m[0][1] ;inv_Matrix.m[1][1] =m.m[1][1] ;inv_Matrix.m[1][2] =m.m[2][1] ;inv_Matrix.m[1][3]=0.0f;
    inv_Matrix.m[2][0] =m.m[0][2] ;inv_Matrix.m[2][1] =m.m[1][2] ;inv_Matrix.m[2][2] =m.m[2][2] ;inv_Matrix.m[2][3]=0.0f;
    inv_Matrix.m[3][0] =-(m.m[3][0] * inv_Matrix.m[0][0] + m.m[3][1] * inv_Matrix.m[1][0] + m.m[3][2] * inv_Matrix.m[2][0]);
    inv_Matrix.m[3][1] =-(m.m[3][0] * inv_Matrix.m[0][1] + m.m[3][1] * inv_Matrix.m[1][1] + m.m[3][2] * inv_Matrix.m[2][1]);
    inv_Matrix.m[3][2] = -(m.m[3][0] * inv_Matrix.m[0][2] + m.m[3][1] * inv_Matrix.m[1][2] + m.m[3][2] * inv_Matrix.m[2][2]);
    inv_Matrix.m[3][3] =1.0f;

    return inv_Matrix;
}


// MAIN ENGINE FUNCTIONS; Called Every Frame:
void InputHandler()
{
    SDL_Event event;
    while(SDL_PollEvent(&event)/*returns 1 if there are events, returns 0 if there arent*/)
    {
        switch(event.type)
        {
        case SDL_QUIT:
            RUNNING = 1;
            break;
        case SDL_KEYDOWN:
            switch(event.key.keysym.sym)
            {
            case SDLK_w:
                camerapos.z ++;
                break;
            case SDLK_s:
                camerapos.z --;
                break;
            case SDLK_a:
                camerapos.x --;
                break;
            case SDLK_d:
                camerapos.x ++;
                break;
            case SDLK_UP:
                camerapos = addVec3f(camerapos,Multiply_vector_scalar(Normalise(LookDir),2.0f));

                break;
            case SDLK_DOWN:
                camerapos = subVec3f(camerapos,Multiply_vector_scalar(Normalise(LookDir),2.0f));

                break;
            case SDLK_LEFT:
                Yaw --;
                break;
            case SDLK_RIGHT:
                Yaw ++;
                break;
            case SDLK_SPACE:
                 camerapos.y --;
                break;
             case SDLK_LSHIFT:
                 camerapos.y ++;
                break;
             default:
                break;
            }
            //fTheta += 1.0f;
            break;
        case SDL_KEYUP:
            switch(event.key.keysym.sym)
            {
            case SDLK_w:
                break;
            case SDLK_s:
                break;
            case SDLK_a:
                break;
            case SDLK_d:
                break;
            default:
                break;
            }
            //fTheta += 1.0f;
            break;

        default:
            break;


        }
    }
}
void ProcessHandler()
{
    //Rotation:


    //fTheta += 0.16;
    //Rotation Y
    matrotY.m[0][0] = cosf(fTheta);
    matrotY.m[0][2] = sinf(fTheta);
    matrotY.m[2][0] = -sinf(fTheta);
    matrotY.m[1][1] = 1.0f;
    matrotY.m[2][2] = cosf(fTheta);
    matrotY.m[3][3] = 1.0f;
    //Rotation Z
    matrotZ.m[0][0] = cosf(fTheta * 0.5f);
    matrotZ.m[0][1] = sinf(fTheta * 0.5f);
    matrotZ.m[1][0] = -sinf(fTheta * 0.5f);
    matrotZ.m[1][1] = cosf(fTheta * 0.5f);
    matrotZ.m[2][2] = 1;
    matrotZ.m[3][3] = 1;

    //Rotation X
    matrotX.m[0][0]=1;
    matrotX.m[1][1]= cosf(fTheta * 0.5f);
    matrotX.m[1][2]= sinf(fTheta * 0.5f);
    matrotX.m[2][1]= -sinf(fTheta * 0.5f);
    matrotX.m[2][2]= cosf(fTheta * 0.5f);
    matrotX.m[3][3]= 1;

}
void DisplayHandler()
{



    //Make transform Matrices

        //Matri4x4 Matrot = create_Unit_Matrix();
        //Matrot = Matrix_Matrix_Multiplication(matrotX,matrotZ);
        //Matrot = Matrix_Matrix_Multiplication(Matrot,matrotY);
/*
        Matri4x4 matCameraRot = Matrix_MakeRotationY(Yaw);
		LookDir = Normalise(Matri4x4MultiplicationHandler(Normalise(Targetv),matCameraRot,0));

        Targetv = addVec3f(camerapos,LookDir);

        Matri4x4 camera_matrix = Matrix_Point_at(camerapos,Targetv,vUp);

        Matri4x4 View_Matrix = Quick_Inversee_Matrix(camera_matrix);
*/



        vUp = (Vec3f){ 1.0f,.0f,1.0f};
        Targetv = (Vec3f){ 0.0f,0.0f,1.0f };
		Matri4x4 matCameraRot = Matrix_MakeRotationY(Radianify(Yaw));
		LookDir = Matri4x4MultiplicationHandler(Targetv,matCameraRot,0);
		Targetv = addVec3f(camerapos, LookDir);
		Matri4x4 camera_matrix = Matrix_Point_at(camerapos, Targetv, vUp);

		// Make view matrix from camera
		Matri4x4 View_Matrix = Quick_Inverse_Matrix(camera_matrix);


    /*color switcher: */  int c = 0;
    for(int i = 0; i < Rohancket.numberOfTriangles; i ++)
    {
        Triangle tri = Rohancket.triangles[i];
        Triangle triprojected, tritranslated, triviewed;

        //Apply Rotation Matrices

        //tri = TriangleTransformHandler(tri,Matrot,0);


        // Translate away from screen
        tritranslated = tri;
        tritranslated = TriangleTranslateHandler(tri,ViewPos);
        // get normal
       Vec3f normal;

        normal = Normalise((GetTriangleNormal(tritranslated)));

        Vec3f CameraRay = GetLine(tritranslated.vertex[0],camerapos);
        //check if triangle is visible
        //printf(" \n [%f] \n",DotProduct(normal,CameraRay));
        if(DotProduct(normal,CameraRay) < 0.0f)
         {
             // Illumination

             Vec3f light_direction;
             light_direction.x = 0.0f;
             light_direction.y = -1.0f;
             light_direction.z = -1.0f;
             light_direction = Normalise(light_direction);

              //printf("normal: (%f, %f, %f)- \n",normal.x, normal.y, normal.z);
              //printf("Light_direction:  (%f, %f, %f) \n",light_direction.x, light_direction.y, light_direction.z);
             float cd = DotProduct(light_direction,normal);
             //printf("cd: %f \n",cd );
            SDL_SetRenderDrawColor(renderer,255 ,255 * cd,255 * cd,255);


            // set from world space to view space
             triviewed = TriangleTransformHandler(tritranslated, View_Matrix,0);
            // Apply projection Matrix
            triprojected = TriangleTransformHandler(triviewed,matProj,1);


            // Since value are between -1 and 1, make them from 0 to 2
            triprojected.vertex[0] = addVec3f(triprojected.vertex[0], (Vec3f){1.0f,1.0f,0.0f});
            triprojected.vertex[1] = addVec3f(triprojected.vertex[1], (Vec3f){1.0f,1.0f,0.0f});
            triprojected.vertex[2] = addVec3f(triprojected.vertex[2], (Vec3f){1.0f,1.0f,0.0f});


            // Then scale them
            Vec3f scale_factor = (Vec3f){0.5f * WIDTH, 0.5f * HEIGHT, 1.0f};
            triprojected.vertex[0] = scaleVec3f(triprojected.vertex[0],scale_factor);
            triprojected.vertex[1] = scaleVec3f(triprojected.vertex[1],scale_factor);
            triprojected.vertex[2] = scaleVec3f(triprojected.vertex[2],scale_factor);


            //Draw the triangle

             //SDL_SetRenderDrawColor(renderer,(commonColors[c].r *(cd)),(commonColors[c].g * (cd)),(commonColors[c].b * (cd)),commonColors[c].a );
            //SDL_SetRenderDrawColor(renderer,255,255,255,255);
            DrawFillTriangHandler(triprojected);
            SDL_SetRenderDrawColor(renderer,255,255,255,255);
            DrawTriangle(triprojected);
        }

    }
     SDL_SetRenderDrawColor(renderer,255,255,255,255);
    SDL_RenderDrawPoint(renderer,WIDTH/2,HEIGHT/2);
    SDL_RenderPresent(renderer);
    SDL_SetRenderDrawColor(renderer,0,0,0,255);
    SDL_RenderClear(renderer);

}




