#include <vector>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#define	DEG2RAD(_x)	((_x)*(3.14159265358979/180.0))
#define	RAD2DEG(_x)	((_x)*(180.0/3.14159265358979))
#define	RAD2ARCMIN(_x)	((_x)*(60.0*180.0/3.14159265358979))

using namespace std;

typedef struct point2d {
	double	x;
	double	y;
} point2d_t;

typedef struct matrix2d {
	double	a11, a12;
	double	a21, a22;
} matrix2d_t;

typedef struct point3d {
	double	x;
	double	y;
	double	z;
} point3d_t;

typedef struct matrix3d {
	double	a11, a12, a13;
	double	a21, a22, a23;
	double	a31, a32, a33;
} matrix3d_t;

class Circumferetial {
	const double		start;			// degree
	const double		end;			// degree
	const double		step;			// degree
	const double		distance_ns;	// mm
	const double		latitude;		// angle in radian
	const bool			is_evaluation;
	const matrix3d_t	upright_matrix;

	public:
	 Circumferetial( const double start0, const double end0, const double step0, const double distance_ns0, const double latitude0, const bool is_evaluation0 );

//	void rotate(point2d_t& out, const double angle, const point2d_t& in);
	void transform(point2d_t& out, const matrix2d_t& matrix, const point2d_t& in);
	void transform(point3d_t& out, const matrix3d_t& matrix, const point3d_t& in);
	void inverse(matrix3d_t& out, const matrix3d_t& in );

	void move(point2d_t& out, const point2d_t& in1, const point2d_t& in2);
	void move(point3d_t& out, const point3d_t& in1, const point3d_t& in2);

	double dot_product(const point3d_t& in1, const point3d_t& in2);
	void vector_product(point3d_t& out, const point3d_t& in1, const point3d_t& in2);
	void normalize( point3d_t& inout );
	void normal_vector(point3d_t& out, const point3d_t& in1, const point3d_t& in2, const point3d_t& in3);

	void roller_on_rail(double& error_south, double& inclination_south, const double length_north /* mm */, const double length_south /* mm */, const double offset_south /* mm */, const double inclination_north /* rad */);
	void rail_on_roller(double& error_south, double& inclination_south, const double length_north /* mm */, const double length_south /* mm */, const double offset_south /* mm */, const double inclination_north /* rad */);
	void roller_on_rail(double& error_south, const double length_north /* mm */, const double offset_south /* mm */, const double inclination_north /* rad */);
	void rail_on_roller(double& error_south, const double length_north /* mm */, const double offset_south /* mm */, const double inclination_north /* rad */);

	// 最小二乗法
	void least_square( double& gradient, double& intercept, const vector<point2d_t>& points, const char * suffix );
	void least_square( point2d_t& center, double& radius, const vector<point2d_t>& points, const char * suffix );

	// 点と直線の距離
	double distance( const double gradient, const double intercept, const point2d_t& point );
};

Circumferetial::Circumferetial(const double start0, const double end0, const double step0, const double distance_ns0, const double latitude0, const bool is_evaluation0 )
:
	start(start0),
	end(end0 + step0*0.5),
	step(step0),
	distance_ns(distance_ns0),
	latitude(latitude0),
	is_evaluation(is_evaluation0),
	upright_matrix({
		1.0 /* a11 */,            0.0 /* a12 */,            0.0 /* a13 */,
		0.0 /* a21 */,  cos(latitude) /* a22 */,  sin(latitude) /* a23 */,
		0.0 /* a31 */, -sin(latitude) /* a32 */,  cos(latitude) /* a33 */
	})
{
}

#if 0
void Circumferetial::rotate(point2d_t& out, const double angle, const point2d_t& in)
{
	matrix2d_t	matrix;

	matrix.a11 = cos( angle );
	matrix.a21 = sin( angle );
	matrix.a12 = -matrix.a21;
	matrix.a22 = matrix.a11;

	transform( out, matrix, in );
}
#endif

void Circumferetial::transform(point2d_t& out, const matrix2d_t& matrix, const point2d_t& in)
{
	double	x, y;

	x = matrix.a11*in.x + matrix.a12*in.y;
	y = matrix.a21*in.x + matrix.a22*in.y;

	out.x = x;
	out.y = y;
}

void Circumferetial::transform(point3d_t& out, const matrix3d_t& matrix, const point3d_t& in)
{
	double	x, y, z;

	x = matrix.a11*in.x + matrix.a12*in.y + matrix.a13*in.z;
	y = matrix.a21*in.x + matrix.a22*in.y + matrix.a23*in.z;
	z = matrix.a31*in.x + matrix.a32*in.y + matrix.a33*in.z;

	out.x = x;
	out.y = y;
	out.z = z;
}

void Circumferetial::inverse(matrix3d_t& out, const matrix3d_t& in )
{
	double	r;
	matrix3d_t	matrix;

	r = in.a11*in.a22*in.a33 + in.a12*in.a23*in.a31 + in.a13*in.a21*in.a32
	  - in.a13*in.a22*in.a31 - in.a12*in.a21*in.a33 - in.a11*in.a23*in.a32;

	matrix.a11 = in.a22*in.a33 - in.a32*in.a23;
	matrix.a12 = in.a13*in.a32 - in.a12*in.a33;
	matrix.a13 = in.a12*in.a23 - in.a13*in.a22;
	matrix.a21 = in.a23*in.a31 - in.a21*in.a33;
	matrix.a22 = in.a11*in.a33 - in.a13*in.a31;
	matrix.a23 = in.a13*in.a21 - in.a11*in.a23;
	matrix.a31 = in.a21*in.a32 - in.a22*in.a31;
	matrix.a32 = in.a12*in.a31 - in.a11*in.a32;
	matrix.a33 = in.a11*in.a22 - in.a12*in.a21;

	out.a11 = matrix.a11/r;
	out.a12 = matrix.a12/r;
	out.a13 = matrix.a13/r;
	out.a21 = matrix.a21/r;
	out.a22 = matrix.a22/r;
	out.a23 = matrix.a23/r;
	out.a31 = matrix.a31/r;
	out.a32 = matrix.a32/r;
	out.a33 = matrix.a33/r;
}

void Circumferetial::move(point2d_t& out, const point2d_t& in1, const point2d_t& in2)
{
	out.x = in1.x + in2.x;
	out.y = in1.y + in2.y;
}

void Circumferetial::move(point3d_t& out, const point3d_t& in1, const point3d_t& in2)
{
	out.x = in1.x + in2.x;
	out.y = in1.y + in2.y;
	out.z = in1.z + in2.z;
}

double Circumferetial::dot_product(const point3d_t& in1, const point3d_t& in2)
{
	return (in1.x*in2.x + in1.y*in2.y + in1.z*in2.z);
}

void Circumferetial::vector_product(point3d_t& out, const point3d_t& in1, const point3d_t& in2)
{
	double	x, y, z;

	x = in1.y*in2.z - in1.z*in2.y;
	y = in1.z*in2.x - in1.x*in2.z;
	z = in1.x*in2.y - in1.y*in2.x;

	out.x = x;
	out.y = y;
	out.z = z;
}

void Circumferetial::normalize( point3d_t& inout )
{
	double	r = sqrt( inout.x*inout.x + inout.y*inout.y + inout.z*inout.z );

	inout.x /= r;
	inout.y /= r;
	inout.z /= r;
}

// calculate a normal vector of the plane from three points.
void Circumferetial::normal_vector(point3d_t& out, const point3d_t& in1, const point3d_t& in2, const point3d_t& in3)
{
	point3d_t	in12;
	point3d_t	in23;

	in12.x = in1.x - in2.x;
	in12.y = in1.y - in2.y;
	in12.z = in1.z - in2.z;

	in23.x = in2.x - in3.x;
	in23.y = in2.y - in3.y;
	in23.z = in2.z - in3.z;

	vector_product( out, in12, in23 );
	normalize( out );
}

// for line
void Circumferetial::least_square( double& gradient, double& intercept, const vector<point2d_t>& points, const char * suffix )
{
	double	sum_x = 0, sum_y = 0;
	double	sum_x_x = 0, sum_x_y = 0;
	double	denominator;

	for(auto& point: points){
		sum_x += point.x;
		sum_y += point.y;
		sum_x_x += point.x*point.x;
		sum_x_y += point.x*point.y;
	}
	denominator = points.size()*sum_x_x - sum_x*sum_x;

	gradient = (points.size()*sum_x_y - sum_x*sum_y)/denominator;
	intercept = (sum_x_x*sum_y - sum_x_y*sum_x)/denominator;

	if( !is_evaluation ){
		cout << "gradient " << suffix << "= " << gradient << endl;
		cout << "intercept " << suffix << "= " << intercept << endl;
		cout << "angle " << suffix << "= " << RAD2DEG( atan(gradient )) << endl;
	}
}

// for circle
void Circumferetial::least_square( point2d_t& center, double& radius, const vector<point2d_t>& points, const char * suffix )
{
	point3d_t	in {0,};
	point3d_t	out;
	matrix3d_t	matrix {0,};

	for(auto& point: points){
		matrix.a11 += point.x*point.x;
		matrix.a12 += point.x*point.y;
		matrix.a13 += point.x;
		matrix.a21 += point.x*point.y;
		matrix.a22 += point.y*point.y;
		matrix.a23 += point.y;
		matrix.a31 += point.x;
		matrix.a32 += point.y;
		matrix.a33 += 1.0;

		in.x -= point.x*point.x*point.x + point.x*point.y*point.y;
		in.y -= point.x*point.x*point.y + point.y*point.y*point.y;
		in.z -= point.x*point.x         + point.y*point.y        ;
	}

	inverse( matrix, matrix );
	transform( out, matrix, in );

	center.x = -0.5*out.x;
	center.y = -0.5*out.y;
	radius = sqrt( center.x*center.x + center.y*center.y - out.z );

	if( !is_evaluation ){
		cout << "center " << suffix << " x= " << center.x << ", y = " << center.y << endl;
		cout << "radius " << suffix << "= " << radius << endl;
	}
}

double Circumferetial::distance( const double gradient, const double intercept, const point2d_t& point )
{
	// 直線 gradient*x - y + intercept = 0
	return ((gradient*point.x - point.y + intercept)/sqrt(gradient*gradient + 1.0));
}

/* rectagon */
void Circumferetial::roller_on_rail(double& error_south, double& inclination_south, const double length_north, const double length_south, const double offset_south, const double inclination_north)
{
	// 北から見ている。z=0として計算。
	// 極軸はZ軸と並行、z軸の正の方向が北
	// x軸の正の方向が西
	// y軸の正の方向が南

	// 振りが0のときの座標たち
	point2d_t	point_ne0;
	point2d_t	point_nw0;
	point2d_t	point_se0;
	point2d_t	point_sw0;

	// 結果の点列
	vector<point2d_t>	points_ne;
	vector<point2d_t>	points_nw;
	vector<point2d_t>	points_se;
	vector<point2d_t>	points_sw;

	// 外接円の半径を求める
	double	circumradius = 0.5*length_north/sin(2.0*inclination_north);
	// x = circumradius*sin(t)
	// y = circumradius*cos(t+pi) + circumradius
	// 外接円の下が(0,0)

	// 振りが0の座標を求める
	point_nw0.x =  0.5*length_north;
	point_nw0.y =  point_nw0.x*tan(inclination_north);
	point_ne0.x = -point_nw0.x;
	point_ne0.y =  point_nw0.y;

	point_sw0.x =  0.5*length_south;
	point_sw0.y =  point_nw0.y + distance_ns*sin(latitude) + offset_south*cos(latitude);
	point_se0.x = -point_sw0.x;
	point_se0.y =  point_sw0.y;

	// 初期値
	for( double angle = start; angle < end; angle += step ){
		// coordinate of roller
		point2d_t	point_ne;
		point2d_t	point_nw;
		point2d_t	point_se;
		point2d_t	point_sw;
		matrix2d_t	matrix;
		point2d_t	point_circumcircle;

		// 外接円上で振り*2の位置を求める
		matrix.a11 = cos( angle );
		matrix.a21 = sin( angle );
		matrix.a12 = -matrix.a21;
		matrix.a22 = matrix.a11;
		point_circumcircle.x = -circumradius*sin(-2.0*angle);
		point_circumcircle.y = -circumradius*(1.0 - cos(-2.0*angle));

		// 振りがangleの座標に移動する
		move( point_ne, point_ne0, point_circumcircle );
		move( point_nw, point_nw0, point_circumcircle );
		move( point_se, point_se0, point_circumcircle );
		move( point_sw, point_sw0, point_circumcircle );

		// 振り回す
		transform( point_ne, matrix, point_ne );
		transform( point_nw, matrix, point_nw );
		transform( point_se, matrix, point_se );
		transform( point_sw, matrix, point_sw );

		// 格納
		points_ne.push_back( point_ne );
		points_nw.push_back( point_nw );
		points_se.push_back( point_se );
		points_sw.push_back( point_sw );
	}

	// 結果をフィッティングした傾きと切片
	double	gradient_ne, intercept_ne;
	double	gradient_nw, intercept_nw;
	double	gradient_se, intercept_se;
	double	gradient_sw, intercept_sw;

	//結果の点列の検算
	least_square( gradient_ne, intercept_ne, points_ne, "NE " );
	least_square( gradient_nw, intercept_nw, points_nw, "NW " );

	//結果の点列のフィッティング
	least_square( gradient_se, intercept_se, points_se, "SE " );
	least_square( gradient_sw, intercept_sw, points_sw, "SW " );

	inclination_south = atan(gradient_sw);
	// y = gradient*x + intercept, 方向ベクトル[1, gradient, 0], (0, intercept, 0)を通る直線
	// y = gradient*x + intercept, 法線ベクトル[gradient, -1, c], (0, intercept, 0)を通る平面

	// 結果表示
	if( !is_evaluation ){
		cout << "Y position of north roller = " << point_ne0.y << endl;
		cout << "Y position of south roller = " << point_se0.y << endl;
		cout << "latitude = " << RAD2DEG(latitude) << endl;
		cout << "circumradius = " << circumradius << endl;
		cout << "inclination of north rail = " << RAD2DEG(inclination_north) << endl;
		cout << "gradient of north rail = " << tan(inclination_north) << endl;
		cout << "inclination of south rail = " << RAD2DEG(inclination_south) << endl;
		cout << "gradient of south rail = " << gradient_sw << endl << endl;

		printf("thrust load on rail for polar axis is %.2f total.\n", sin(latitude) );
		printf("radial load on north rail for polar axis is %.2f per roller.\n", 0.25*cos(latitude)/cos(inclination_north) );
		printf("radial load on south rail for polar axis is %.2f per roller.\n\n", 0.25*cos(latitude)/cos(inclination_south) );
	}

	double	zs = -distance_ns*cos(latitude);
	FILE *datafile = NULL;
	char	filename[1024] = {0};
	if( !is_evaluation ){
		// y = gradient_ne*x + intercept_ne + c_ne*(z - z0) の平面が
		// (x, y, z) = (0, intercept_ne, z0) + t*(0, tan(latitude), -1) という直線を含む

		// 法線ベクトルと方向ベクトルの内積 = 0 なので
		// -tan(latitude) - c_ne = 0
		// 平面が直線を含むので
		// intercept_ne + t*tan(latitude) = intercept_ne + c_ne*(z0 - t - z0)

		// いずれにせよ
		// c_ne = -tan(latitude) となるので、平面は、
		// y = gradient_ne*x + intercept_ne -tan(latitude)*(z - z0)
		// -gradient_ne*x + (y - intercept_ne) + tan(latitude)*(z - z0) = 0
		double	tan_lat = tan(latitude);
		double	rr;
		double	dot_product;
		printf("normal vecror of NE = (%8.3f, %8.3f, %8.3f)\n", -gradient_ne, 1.0, tan_lat );
		rr = gradient_ne*gradient_ne + 1.0 + tan_lat*tan_lat;
		dot_product = gradient_ne*gradient_nw + 1.0 + tan_lat*tan_lat;
		printf("angle between normal vecrors = %8.3f\n", RAD2DEG( acos(dot_product/rr) ) );

		printf("normal vecror of SE = (%8.3f, %8.3f, %8.3f)\n", -gradient_se, 1.0, tan_lat );
		rr = gradient_se*gradient_se + 1.0 + tan_lat*tan_lat;
		dot_product = gradient_se*gradient_sw + 1.0 + tan_lat*tan_lat;
		printf("angle between normal vecrors = %8.3f\n", RAD2DEG( acos(dot_product/rr) ) );

		snprintf( filename, sizeof filename, "roller_on_rail-l%.0f-s%.0f-n%.0f-ns%.0f-i%.0f.txt",
			RAD2DEG(latitude), length_south, length_north, distance_ns, RAD2DEG(inclination_north) );
		datafile = fopen(filename, "w");

		printf( R"(reset;set term qt size 1200,900;set view equal xyz)" "\n" );

		printf( R"(splot )" );
		printf( R"('%s' using 1:2:3 title "North east", )", filename );
		printf( R"('%s' using 4:5:6 title "North west", )", filename );
		printf( R"('%s' using 7:8:9 title "South east", )", filename );
		printf( R"('%s' using 10:11:12 title "South west", )", filename );
		printf( R"('%s' using ($1+$4)/2:($2+$5)/2:($3+$6)/2 title "North center", )", filename );
		printf( R"('%s' using ($7+$10)/2:($8+$11)/2:($9+$12)/2 title "South center", )", filename);
		printf( R"(0 title "North plane", %.3f title "South plane")" "\n", zs );

		printf( R"(splot )" );
		printf( R"('%s' using 1:2:3 title "North east", )", filename );
		printf( R"('%s' using 4:5:6 title "North west", )", filename );
		printf( R"('%s' using 7:8:9 title "South east", )", filename );
		printf( R"('%s' using 10:11:12 title "South west", )", filename );
		printf( R"('%s' using ($1+$4)/2:($2+$5)/2:($3+$6)/2 title "North center", )", filename );
		printf( R"('%s' using ($7+$10)/2:($8+$11)/2:($9+$12)/2 title "South center", )", filename);
		printf( R"((%.3f*x - (y - %.3f))/%.3f title "North east plane", )", gradient_ne, intercept_ne, tan_lat );
		printf( R"((%.3f*x - (y - %.3f))/%.3f title "North west plane", )", gradient_nw, intercept_nw, tan_lat );
		printf( R"((%.3f*x - (y - %.3f))/%.3f + %.3f title "South east plane", )", gradient_se, intercept_se, tan_lat, zs );
		printf( R"((%.3f*x - (y - %.3f))/%.3f + %.3f title "South west plane")" "\n", gradient_sw, intercept_sw, tan_lat, zs );

		printf( R"(splot )" );
		printf( R"('%s' using 13:14:15 title "North east", )", filename );
		printf( R"('%s' using 16:17:18 title "North west", )", filename );
		printf( R"('%s' using 19:20:21 title "South east", )", filename );
		printf( R"('%s' using 22:23:24 title "South west", )", filename );
		printf( R"('%s' using ($13+$16)/2:($14+$17)/2:($15+$18)/2 title "North center", )", filename );
		printf( R"('%s' using ($19+$22)/2:($20+$23)/2:($21+$24)/2 title "South center")" "\n", filename);

		cout << "angle, ne.x, ne.y, nw.x, nw.y, se.x, se.y, se.error, sw.x, sw.y, sw.error" << endl;
//		cout << "angle, ne.x, ne.y, ne.error, nw.x, nw.y, nw.error, se.x, se.y, se.error, sw.x, sw.y, sw.error" << endl;
	}

	int	i = 0;
	error_south = 0;
	for( double angle = start; angle < end; angle += step ){
		double	err;
		if( !is_evaluation ){
			point3d_t	upright_ne, upright_nw, upright_se, upright_sw;
			printf("%5.1f,", RAD2DEG(angle) );

			// 2D plot
			printf("%7.1f,%7.1f,", points_ne[i].x, points_ne[i].y );
//			printf("%7.3f,", distance( gradient_ne, intercept_ne, points_ne[i] ) );
			printf("%7.1f,%7.1f,", points_nw[i].x, points_nw[i].y );
//			printf("%7.3f,", distance( gradient_nw, intercept_nw, points_nw[i] ) );
			printf("%7.1f,%7.1f,", points_se[i].x, points_se[i].y );
			printf("%7.3f,", distance( gradient_se, intercept_se, points_se[i] ) );
			printf("%7.1f,%7.1f,", points_sw[i].x, points_sw[i].y );
			printf("%7.3f\n", distance( gradient_sw, intercept_sw, points_sw[i] ) );

			// 3D plot
			upright_ne.x = points_ne[i].x; upright_ne.y = points_ne[i].y; upright_ne.z = 0.0;
			upright_nw.x = points_nw[i].x; upright_nw.y = points_nw[i].y; upright_nw.z = 0.0;
			upright_se.x = points_se[i].x; upright_se.y = points_se[i].y; upright_se.z = zs;
			upright_sw.x = points_sw[i].x; upright_sw.y = points_sw[i].y; upright_sw.z = zs;
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_ne.x, upright_ne.y, upright_ne.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_nw.x, upright_nw.y, upright_nw.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_se.x, upright_se.y, upright_se.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_sw.x, upright_sw.y, upright_sw.z );

			// 3D plot
			transform( upright_ne, upright_matrix, upright_ne );
			transform( upright_nw, upright_matrix, upright_nw );
			transform( upright_se, upright_matrix, upright_se );
			transform( upright_sw, upright_matrix, upright_sw );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_ne.x, upright_ne.y, upright_ne.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_nw.x, upright_nw.y, upright_nw.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_se.x, upright_se.y, upright_se.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f\n", upright_sw.x, upright_sw.y, upright_sw.z );
		}

		err = abs( distance( gradient_sw, intercept_sw, points_sw[i] ) );
		if( error_south < err ){
			error_south = err;
		}

		i++;
	}

	if( !is_evaluation ){
		fclose(datafile);

		vector<point2d_t>::iterator	itr_first = points_ne.begin();
		vector<point2d_t>::reverse_iterator	itr_last = points_ne.rbegin();
		printf("North roller travel = %7.3f", hypot( (*itr_first).x - (*itr_last).x, (*itr_first).y - (*itr_last).y ) );
		printf(", up = %7.3f", hypot( (*itr_first).x - point_ne0.x, (*itr_first).y - point_ne0.y ) );
		printf(", down = %7.3f\n", hypot( (*itr_last).x - point_ne0.x, (*itr_last).y - point_ne0.y ) );
		printf("North center x-amplitude = %7.3f", abs( (*itr_first).x - (*itr_last).x ) );
		printf(", y-amplitude = %7.3f\n", abs( 0.5*((*itr_first).y + (*itr_last).y) - point_ne0.y  ) );

		itr_first = points_se.begin();
		itr_last = points_se.rbegin();
		printf("South roller travel = %7.3f", hypot( (*itr_first).x - (*itr_last).x, (*itr_first).y - (*itr_last).y ) );
		printf(", up = %7.3f", hypot( (*itr_first).x - point_se0.x, (*itr_first).y - point_se0.y ) );
		printf(", down = %7.3f\n", hypot( (*itr_last).x - point_se0.x, (*itr_last).y - point_se0.y ) );
		printf("South center x-amplitude = %7.3f", abs( (*itr_first).x - (*itr_last).x ) );
		printf(", y-amplitude = %7.3f\n", abs( 0.5*((*itr_first).y + (*itr_last).y) - point_se0.y  ) );
	}
}

/* triangle */
void Circumferetial::roller_on_rail(double& error_south, const double length_north, const double offset_south, const double inclination_north)
{
	// 北から見ている。z=0として計算。
	// 極軸はZ軸と並行、z軸の正の方向が北
	// x軸の正の方向が西
	// y軸の正の方向が南

	// 振りが0のときの座標たち
	point2d_t	point_ne0;
	point2d_t	point_nw0;
	point2d_t	point_sc0;
	point2d_t	point_so0;

	// 結果の点列
	vector<point2d_t>	points_ne;
	vector<point2d_t>	points_nw;
	vector<point2d_t>	points_sc;
	vector<point2d_t>	points_so;

	// 外接円の半径を求める
	double	circumradius = 0.5*length_north/sin(2.0*inclination_north);
	// x = circumradius*sin(t)
	// y = circumradius*cos(t+pi) + circumradius
	// 外接円の下が(0,0)

	// 振りが0の座標を求める
	point_nw0.x =  0.5*length_north;
	point_nw0.y =  point_nw0.x*tan(inclination_north);
	point_ne0.x = -point_nw0.x;
	point_ne0.y =  point_nw0.y;

	point_sc0.x =  0.0;
	point_sc0.y = point_nw0.y + distance_ns*sin(latitude);
	point_so0.x =  0.0;
	point_so0.y = point_sc0.y + offset_south*cos(latitude);

	// 初期値
	for( double angle = start; angle < end; angle += step ){
		// coordinate of roller
		point2d_t	point_ne;
		point2d_t	point_nw;
		point2d_t	point_sc;
		point2d_t	point_so;
		matrix2d_t	matrix;
		point2d_t	point_circumcircle;

		// 外接円上で振り*2の位置を求める
		matrix.a11 = cos( angle );
		matrix.a21 = sin( angle );
		matrix.a12 = -matrix.a21;
		matrix.a22 = matrix.a11;
		point_circumcircle.x = -circumradius*sin(-2.0*angle);
		point_circumcircle.y = -circumradius*(1.0 - cos(-2.0*angle));

		// 振りがangleの座標に移動する
		move( point_ne, point_ne0, point_circumcircle );
		move( point_nw, point_nw0, point_circumcircle );
		move( point_sc, point_sc0, point_circumcircle );
		move( point_so, point_so0, point_circumcircle );

		// 振り回す
		transform( point_ne, matrix, point_ne );
		transform( point_nw, matrix, point_nw );
		transform( point_sc, matrix, point_sc );
		transform( point_so, matrix, point_so );

		// 格納
		points_ne.push_back( point_ne );
		points_nw.push_back( point_nw );
		points_sc.push_back( point_sc );
		points_so.push_back( point_so );
	}

	//結果の点列のフィッティング
	point2d_t	center_sc, center_so;
	double		radius_sc, radius_so;

	least_square( center_sc, radius_sc, points_sc, "south center " );
	least_square( center_so, radius_so, points_so, "south center with offset " );

	// 結果表示
	if( !is_evaluation ){
		cout << "Y position of north roller = " << point_ne0.y << endl;
		cout << "Y position of south center = " << point_sc0.y << endl;
		cout << "Y position of south center with offset= " << point_so0.y << endl;
		cout << "latitude = " << RAD2DEG(latitude) << endl;
		cout << "circumradius = " << circumradius << endl;
		cout << "inclination of north rail = " << RAD2DEG(inclination_north) << endl;
		cout << "gradient of north rail = " << tan(inclination_north) << endl;

		printf("thrust load on rail for polar axis is %.2f total.\n", sin(latitude) );
		printf("radial load on north rail for polar axis is %.2f per roller.\n", 0.25*cos(latitude)/cos(inclination_north) );
		printf("radial load on south center for polar axis is %.2f.\n\n", 0.5*cos(latitude) );
	}

	double	zs = -distance_ns*cos(latitude);
	FILE *datafile = NULL;
	char	filename[1024] = {0};
	if( !is_evaluation ){
#if 0
		snprintf( filename, sizeof filename, "roller_on_rail-l%.0f-o%.0f-n%.0f-ns%.0f-i%.0f.txt",
			RAD2DEG(latitude), offset_south, length_north, distance_ns, RAD2DEG(inclination_north) );
#else
		snprintf( filename, sizeof filename, "roller_on_rail-n%.0f.txt", length_north );
#endif
		datafile = fopen(filename, "w");

		printf( R"(reset;set term qt size 1200,900;set view equal xyz)" "\n" );

		printf( R"(splot )" );
		printf( R"('%s' using 1:2:3 title "North east", )", filename );
		printf( R"('%s' using 4:5:6 title "North west", )", filename );
		printf( R"('%s' using 7:8:9 title "South center", )", filename );
		printf( R"('%s' using 10:11:12 title "South center with offset", )", filename );
		printf( R"('%s' using ($1+$4)/2:($2+$5)/2:($3+$6)/2 title "North center", )", filename);
		printf( R"(0 title "North plane", %.3f title "South plane")" "\n", zs );

		printf( R"(splot )" );
		printf( R"('%s' using 13:14:15 title "North east", )", filename );
		printf( R"('%s' using 16:17:18 title "North west", )", filename );
		printf( R"('%s' using 19:20:21 title "South center", )", filename );
		printf( R"('%s' using 22:23:24 title "South center with offset", )", filename );
		printf( R"('%s' using ($13+$16)/2:($14+$17)/2:($15+$18)/2 title "North center")" "\n", filename);

		cout << "angle,   ne.x,   ne.y,   nw.x,   nw.y,   sc.x,   sc.y, sc.err,   so.x,   so.y, so.err" << endl;
	}

	int	i = 0;
	error_south = 0;
	for( double angle = start; angle < end; angle += step ){
		point3d_t	upright_ne, upright_nw, upright_sc, upright_so;
		double	err_so, err_sc;

		upright_ne.x = points_ne[i].x; upright_ne.y = points_ne[i].y; upright_ne.z = 0.0;
		upright_nw.x = points_nw[i].x; upright_nw.y = points_nw[i].y; upright_nw.z = 0.0;
		upright_sc.x = points_sc[i].x; upright_sc.y = points_sc[i].y; upright_sc.z = zs;
		upright_so.x = points_so[i].x; upright_so.y = points_so[i].y; upright_so.z = zs;
#if 0
		point3d_t	slide_so, normal_original, normal_slide;
		slide_so.x = points_so[i].x; slide_so.y = point_so0.y; slide_so.z = zs;
		normal_vector( normal_original, upright_ne, upright_nw, upright_so );
		normal_vector( normal_slide, upright_ne, upright_nw, slide_so );
		err_so = RAD2ARCMIN( acos( dot_product( normal_original, normal_slide ) ) );
#else
		err_sc = hypot( points_sc[i].x - center_sc.x, points_sc[i].y - center_sc.y ) - radius_sc;
		err_so = hypot( points_so[i].x - center_so.x, points_so[i].y - center_so.y ) - radius_so;
#endif

		if( !is_evaluation ){
			// 2D plot
			printf("%5.1f,", RAD2DEG(angle) );
			printf("%7.1f,%7.1f,", points_ne[i].x, points_ne[i].y );
			printf("%7.1f,%7.1f,", points_nw[i].x, points_nw[i].y );
			printf("%7.1f,%7.1f,", points_sc[i].x, points_sc[i].y );
			printf("%7.3f,", err_sc );
			printf("%7.1f,%7.1f,", points_so[i].x, points_so[i].y );
			printf("%7.3f\n", err_so );

			// 3D plot
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_ne.x, upright_ne.y, upright_ne.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_nw.x, upright_nw.y, upright_nw.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_sc.x, upright_sc.y, upright_sc.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_so.x, upright_so.y, upright_so.z );

			// 3D plot
			transform( upright_ne, upright_matrix, upright_ne );
			transform( upright_nw, upright_matrix, upright_nw );
			transform( upright_sc, upright_matrix, upright_sc );
			transform( upright_so, upright_matrix, upright_so );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_ne.x, upright_ne.y, upright_ne.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_nw.x, upright_nw.y, upright_nw.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_sc.x, upright_sc.y, upright_sc.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f\n", upright_so.x, upright_so.y, upright_so.z );
		}

		if( error_south < abs(err_so) ){
			error_south = err_so;
		}

		i++;
	}

	if( !is_evaluation ){
		fclose(datafile);

		vector<point2d_t>::iterator	itr_first = points_ne.begin();
		vector<point2d_t>::reverse_iterator	itr_last = points_ne.rbegin();
		printf("North roller travel = %7.3f", hypot( (*itr_first).x - (*itr_last).x, (*itr_first).y - (*itr_last).y ) );
		printf(", up = %7.3f", hypot( (*itr_first).x - point_ne0.x, (*itr_first).y - point_ne0.y ) );
		printf(", down = %7.3f\n", hypot( (*itr_last).x - point_ne0.x, (*itr_last).y - point_ne0.y ) );
		printf("North center x-amplitude = %7.3f", abs( (*itr_first).x - (*itr_last).x ) );
		printf(", y-amplitude = %7.3f\n", abs( 0.5*((*itr_first).y + (*itr_last).y) - point_ne0.y  ) );

		itr_first = points_sc.begin();
		itr_last = points_sc.rbegin();
		printf("South center x-amplitude = %7.3f", abs( (*itr_first).x - (*itr_last).x ) );
		printf(", y-amplitude = %7.3f\n", abs( 0.5*((*itr_first).y + (*itr_last).y) - point_sc0.y  ) );

		itr_first = points_so.begin();
		itr_last = points_so.rbegin();
		printf("South center with offset x-amplitude = %7.3f", abs( (*itr_first).x - (*itr_last).x ) );
		printf(", y-amplitude = %7.3f\n", abs( 0.5*((*itr_first).y + (*itr_last).y) - point_so0.y  ) );
	}
}

/* rectagon */
void Circumferetial::rail_on_roller(double& error_south, double& inclination_south, const double length_north, const double length_south, const double offset_south, const double inclination_north)
{
	// 北から見ている。z=0として計算。
	// 極軸はZ軸と並行、z軸の正の方向が北
	// x軸の正の方向が西
	// y軸の正の方向が南

	// 振りが0のときの座標たち
	point2d_t	point_nw0;
	point2d_t	point_nc0;
	point2d_t	point_ne0;
	point2d_t	point_sw0;
	point2d_t	point_sc0;
	point2d_t	point_se0;

	// 結果の点列
	vector<point2d_t>	points_ne;
	vector<point2d_t>	points_nc;
	vector<point2d_t>	points_nw;
	vector<point2d_t>	points_se;
	vector<point2d_t>	points_sc;
	vector<point2d_t>	points_sw;

	// 外接円の半径を求める
	double	circumradius = 0.5*length_north/sin(2.0*inclination_north);
	// x = circumradius*sin(t)
	// y = circumradius*cos(t+pi) + circumradius
	// 外接円の下が(0,0)

	// 振りが0の座標を求める
	point_nw0.x =  0.5*length_north;
	point_nw0.y = point_nw0.x*tan(inclination_north);
	point_nc0.x =  0.0;
	point_nc0.y = point_nw0.y;
	point_ne0.x = -0.5*length_north;
	point_ne0.y = point_nw0.y;

	point_sw0.x =  0.5*length_south;
	point_sw0.y = point_nw0.y + distance_ns*sin(latitude) + offset_south*cos(latitude);
	point_sc0.x =  0.0;
	point_sc0.y = point_sw0.y;
	point_se0.x = -0.5*length_south;
	point_se0.y = point_sw0.y;

	// 初期値
	for( double angle = start; angle < end; angle += step ){
		// coordinate on rail
		point2d_t	point_ne;
		point2d_t	point_nw;
		point2d_t	point_se;
		point2d_t	point_sw;
		matrix2d_t	matrix;
		point2d_t	point_circumcircle;

		// 外接円上で振り*2の位置を求める
		matrix.a11 = cos( angle );
		matrix.a21 = sin( angle );
		matrix.a12 = -matrix.a21;
		matrix.a22 = matrix.a11;
		point_circumcircle.x = -circumradius*sin(-2.0*angle);
		point_circumcircle.y = -circumradius*(1.0 - cos(-2.0*angle));

		// 振りがangleの座標に移動する
		move( point_ne, point_ne0, point_circumcircle );
		move( point_nw, point_nw0, point_circumcircle );
		move( point_se, point_se0, point_circumcircle );
		move( point_sw, point_sw0, point_circumcircle );

		// 振り回す
		transform( point_ne, matrix, point_ne );
		transform( point_nw, matrix, point_nw );
		transform( point_se, matrix, point_se );
		transform( point_sw, matrix, point_sw );

		// 格納
		points_ne.push_back( point_ne );
		points_nw.push_back( point_nw );
		points_se.push_back( point_se );
		points_sw.push_back( point_sw );

		// center point
		point2d_t	point_nc;
		point2d_t	point_sc;

		matrix.a21 *= -1.0;
		matrix.a12 *= -1.0;
		point_circumcircle.x *= -1.0;
		point_circumcircle.y *= -1.0;

		// 振り回す
		transform( point_nc, matrix, point_nc0 );
		transform( point_sc, matrix, point_sc0 );

		// 振りがangleの座標に移動する
		move( point_nc, point_nc, point_circumcircle );
		move( point_sc, point_sc, point_circumcircle );

		// 格納
		points_nc.push_back( point_nc );
		points_sc.push_back( point_sc );
	}

	// 結果をフィッティングした傾きと切片
	double	gradient_ne, intercept_ne;
	double	gradient_nw, intercept_nw;
	double	gradient_se, intercept_se;
	double	gradient_sw, intercept_sw;

	//結果の点列の検算
	least_square( gradient_ne, intercept_ne, points_ne, "NE " );
	least_square( gradient_nw, intercept_nw, points_nw, "NW " );

	//結果の点列のフィッティング
	least_square( gradient_se, intercept_se, points_se, "SE " );
	least_square( gradient_sw, intercept_sw, points_sw, "SW " );

	inclination_south = atan(gradient_sw);
	// y = gradient*x + intercept, 方向ベクトル[1, gradient, 0], (0, intercept, 0)を通る直線
	// y = gradient*x + intercept, 法線ベクトル[gradient, -1, c], (0, intercept, 0)を通る平面

	// 結果表示
	if( !is_evaluation ){
		cout << "Y position of north roller = " << point_ne0.y << endl;
		cout << "Y position of south roller = " << point_se0.y << endl;
		cout << "latitude = " << RAD2DEG(latitude) << endl;
		cout << "circumradius = " << circumradius << endl;
		cout << "inclination of north rail = " << RAD2DEG(inclination_north) << endl;
		cout << "gradient of north rail = " << tan(inclination_north) << endl;
		cout << "inclination of south rail = " << RAD2DEG(inclination_south) << endl;
		cout << "gradient of south rail = " << gradient_sw << endl << endl;
	}

	double	zs = -distance_ns*cos(latitude);
	FILE *datafile = NULL;
	char	filename[1024] = {0};
	if( !is_evaluation ){
		snprintf( filename, sizeof filename, "rail_on_roller-l%.0f-s%.0f-n%.0f-ns%.0f-i%.0f.txt",
			RAD2DEG(latitude), length_south, length_north, distance_ns, RAD2DEG(inclination_north) );
		datafile = fopen(filename, "w");

		printf( R"(reset;set term qt size 1200,900;set view equal xyz)" "\n" );

		printf( R"(splot )" );
		printf( R"('%s' using 1:2:3 title "North east rail", )", filename );
		printf( R"('%s' using 4:5:6 title "North west rail", )", filename );
		printf( R"('%s' using 7:8:9 title "South east rail", )", filename );
		printf( R"('%s' using 10:11:12 title "South west rail", )", filename );
		printf( R"('%s' using 13:14:15 title "North center", )", filename );
		printf( R"('%s' using 16:17:18 title "South center", )", filename);
		printf( R"(0 title "North plane", %.3f title "South plane")" "\n", zs );

		printf( R"(splot )" );
		printf( R"('%s' using 19:20:21 title "North east rail", )", filename );
		printf( R"('%s' using 22:23:24 title "North west rail", )", filename );
		printf( R"('%s' using 25:26:27 title "South east rail", )", filename );
		printf( R"('%s' using 28:29:30 title "South west rail", )", filename );
		printf( R"('%s' using 31:32:33 title "North center", )", filename );
		printf( R"('%s' using 34:35:36 title "South center", )" "\n", filename );

		cout << "angle,   ne.x,   ne.y,   nw.x,   nw.y,   se.x,   se.y, se.err,   sw.x,   sw.y, sw.err,   nc,x,   nc,y,   sc,x,   sc,y" << endl;
//		cout << "angle,   ne.x,   ne.y, ne.err,   nw.x,   nw.y, nw.err,   se.x,   se.y, se.err,   sw.x,   sw.y, sw.err,   nc,x,   nc,y,   sc,x,   sc,y" << endl;
	}

	int	i = 0;
	error_south = 0;
	for( double angle = start; angle < end; angle += step ){
		double	err;
		if( !is_evaluation ){
			point3d_t	upright_ne, upright_nw, upright_se, upright_sw;
			point3d_t	upright_nc, upright_sc;
			printf("%5.1f,", RAD2DEG(angle) );

			// 2D plot
			printf("%7.1f,%7.1f,", points_ne[i].x, points_ne[i].y );
//			printf("%7.3f,", distance( gradient_ne, intercept_ne, points_ne[i] ) );
			printf("%7.1f,%7.1f,", points_nw[i].x, points_nw[i].y );
//			printf("%7.3f,", distance( gradient_nw, intercept_nw, points_nw[i] ) );
			printf("%7.1f,%7.1f,", points_se[i].x, points_se[i].y );
			printf("%7.3f,", distance( gradient_se, intercept_se, points_se[i] ) );
			printf("%7.1f,%7.1f,", points_sw[i].x, points_sw[i].y );
			printf("%7.3f,", distance( gradient_sw, intercept_sw, points_sw[i] ) );
			printf("%7.1f,%7.1f,", points_nc[i].x, points_nc[i].y );
			printf("%7.1f,%7.1f\n", points_sc[i].x, points_sc[i].y );

			// 3D plot
			upright_ne.x = points_ne[i].x; upright_ne.y = points_ne[i].y; upright_ne.z = 0.0;
			upright_nc.x = points_nc[i].x; upright_nc.y = points_nc[i].y; upright_nc.z = 0.0;
			upright_nw.x = points_nw[i].x; upright_nw.y = points_nw[i].y; upright_nw.z = 0.0;
			upright_se.x = points_se[i].x; upright_se.y = points_se[i].y; upright_se.z = zs;
			upright_sc.x = points_sc[i].x; upright_sc.y = points_sc[i].y; upright_sc.z = zs;
			upright_sw.x = points_sw[i].x; upright_sw.y = points_sw[i].y; upright_sw.z = zs;
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_ne.x, upright_ne.y, upright_ne.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_nw.x, upright_nw.y, upright_nw.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_se.x, upright_se.y, upright_se.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_sw.x, upright_sw.y, upright_sw.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_nc.x, upright_nc.y, upright_nc.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_sc.x, upright_sc.y, upright_sc.z );

			// 3D plot
			transform( upright_ne, upright_matrix, upright_ne );
			transform( upright_nc, upright_matrix, upright_nc );
			transform( upright_nw, upright_matrix, upright_nw );
			transform( upright_se, upright_matrix, upright_se );
			transform( upright_sc, upright_matrix, upright_sc );
			transform( upright_sw, upright_matrix, upright_sw );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_ne.x, upright_ne.y, upright_ne.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_nw.x, upright_nw.y, upright_nw.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_se.x, upright_se.y, upright_se.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_sw.x, upright_sw.y, upright_sw.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_nc.x, upright_nc.y, upright_nc.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f\n", upright_sc.x, upright_sc.y, upright_sc.z );
		}

		err = abs( distance( gradient_sw, intercept_sw, points_sw[i] ) );
		if( error_south < err ){
			error_south = err;
		}

		i++;
	}

	if( !is_evaluation ){
		fclose(datafile);

		vector<point2d_t>::iterator	itr_first = points_ne.begin();
		vector<point2d_t>::reverse_iterator	itr_last = points_ne.rbegin();
		printf("North rail length = %7.3f", hypot( (*itr_first).x - (*itr_last).x, (*itr_first).y - (*itr_last).y ) );
		printf(", up = %7.3f", hypot( (*itr_first).x - point_ne0.x, (*itr_first).y - point_ne0.y ) );
		printf(", down = %7.3f\n", hypot( (*itr_last).x - point_ne0.x, (*itr_last).y - point_ne0.y ) );

		itr_first = points_se.begin();
		itr_last = points_se.rbegin();
		printf("South rail length = %7.3f", hypot( (*itr_first).x - (*itr_last).x, (*itr_first).y - (*itr_last).y ) );
		printf(", up = %7.3f", hypot( (*itr_first).x - point_se0.x, (*itr_first).y - point_se0.y ) );
		printf(", down = %7.3f\n", hypot( (*itr_last).x - point_se0.x, (*itr_last).y - point_se0.y ) );

		itr_first = points_nc.begin();
		itr_last = points_nc.rbegin();
		printf("North center x-amplitude = %7.3f", abs( (*itr_first).x - (*itr_last).x ) );
		printf(", y-amplitude = %7.3f\n", abs( 0.5*((*itr_first).y + (*itr_last).y) - point_nc0.y  ) );

		itr_first = points_sc.begin();
		itr_last = points_sc.rbegin();
		printf("South center x-amplitude = %7.3f", abs( (*itr_first).x - (*itr_last).x ) );
		printf(", y-amplitude = %7.3f\n", abs( 0.5*((*itr_first).y + (*itr_last).y) - point_sc0.y  ) );
	}
}

/* triangle */
void Circumferetial::rail_on_roller(double& error_south, const double length_north, const double offset_south, const double inclination_north)
{
	// 北から見ている。z=0として計算。
	// 極軸はZ軸と並行、z軸の正の方向が北
	// x軸の正の方向が西
	// y軸の正の方向が南

	// 振りが0のときの座標たち
	point2d_t	point_nw0;
	point2d_t	point_nc0;
	point2d_t	point_ne0;
	point2d_t	point_sc0;
	point2d_t	point_so0;

	// 結果の点列
	vector<point2d_t>	points_ne;
	vector<point2d_t>	points_nc;
	vector<point2d_t>	points_nw;
	vector<point2d_t>	points_sc;
	vector<point2d_t>	points_so;

	// 外接円の半径を求める
	double	circumradius = 0.5*length_north/sin(2.0*inclination_north);
	// x = circumradius*sin(t)
	// y = circumradius*cos(t+pi) + circumradius
	// 外接円の下が(0,0)

	// 振りが0の座標を求める
	point_nw0.x =  0.5*length_north;
	point_nw0.y = point_nw0.x*tan(inclination_north);
	point_nc0.x =  0.0;
	point_nc0.y = point_nw0.y;
	point_ne0.x = -0.5*length_north;
	point_ne0.y = point_nw0.y;

	point_sc0.x =  0.0;
	point_sc0.y = point_nw0.y + distance_ns*sin(latitude);
	point_so0.x =  0.0;
	point_so0.y = point_sc0.y + offset_south*cos(latitude);

	// 初期値
	for( double angle = start; angle < end; angle += step ){
		// coordinate on rail
		point2d_t	point_ne;
		point2d_t	point_nw;
		matrix2d_t	matrix;
		point2d_t	point_circumcircle;

		// 外接円上で振り*2の位置を求める
		matrix.a11 = cos( angle );
		matrix.a21 = sin( angle );
		matrix.a12 = -matrix.a21;
		matrix.a22 = matrix.a11;
		point_circumcircle.x = -circumradius*sin(-2.0*angle);
		point_circumcircle.y = -circumradius*(1.0 - cos(-2.0*angle));

		// 振りがangleの座標に移動する
		move( point_ne, point_ne0, point_circumcircle );
		move( point_nw, point_nw0, point_circumcircle );

		// 振り回す
		transform( point_ne, matrix, point_ne );
		transform( point_nw, matrix, point_nw );

		// 格納
		points_ne.push_back( point_ne );
		points_nw.push_back( point_nw );

		// center point
		point2d_t	point_nc;
		point2d_t	point_sc;
		point2d_t	point_so;

		matrix.a21 *= -1.0;
		matrix.a12 *= -1.0;
		point_circumcircle.x *= -1.0;
		point_circumcircle.y *= -1.0;

		// 振り回す
		transform( point_nc, matrix, point_nc0 );
		transform( point_sc, matrix, point_sc0 );
		transform( point_so, matrix, point_so0 );

		// 振りがangleの座標に移動する
		move( point_nc, point_nc, point_circumcircle );
		move( point_sc, point_sc, point_circumcircle );
		move( point_so, point_so, point_circumcircle );

		// 格納
		points_nc.push_back( point_nc );
		points_sc.push_back( point_sc );
		points_so.push_back( point_so );
	}

	//結果の点列のフィッティング
	point2d_t	center_sc, center_so;
	double		radius_sc, radius_so;

	least_square( center_sc, radius_sc, points_sc, "south center " );
	least_square( center_so, radius_so, points_so, "south center with offset " );

	// 結果表示
	if( !is_evaluation ){
		cout << "Y position of north roller = " << point_ne0.y << endl;
		cout << "Y position of south center = " << point_sc0.y << endl;
		cout << "Y position of south center with offset= " << point_so0.y << endl;
		cout << "latitude = " << RAD2DEG(latitude) << endl;
		cout << "circumradius = " << circumradius << endl;
		cout << "inclination of north rail = " << RAD2DEG(inclination_north) << endl;
		cout << "gradient of north rail = " << tan(inclination_north) << endl;
	}

	double	zs = -distance_ns*cos(latitude);
	FILE *datafile = NULL;
	char	filename[1024] = {0};
	if( !is_evaluation ){
#if 0
		snprintf( filename, sizeof filename, "rail_on_roller-l%.0f-s%.0f-n%.0f-ns%.0f-i%.0f.txt",
			RAD2DEG(latitude), length_south, length_north, distance_ns, RAD2DEG(inclination_north) );
#else
		snprintf( filename, sizeof filename, "rail_on_roller-n%.0f.txt", length_north );
#endif
		datafile = fopen(filename, "w");

		printf( R"(reset;set term qt size 1200,900;set view equal xyz)" "\n" );

		printf( R"(splot )" );
		printf( R"('%s' using 1:2:3 title "North east rail", )", filename );
		printf( R"('%s' using 4:5:6 title "North west rail", )", filename );
		printf( R"('%s' using 7:8:9 title "North center", )", filename );
		printf( R"('%s' using 10:11:12 title "South center", )", filename );
		printf( R"('%s' using 13:14:15 title "South center with offset", )", filename );
		printf( R"(0 title "North plane", %.3f title "South plane")" "\n", zs );

		printf( R"(splot )" );
		printf( R"('%s' using 16:17:18 title "North east rail", )", filename);
		printf( R"('%s' using 19:20:21 title "North west rail", )", filename );
		printf( R"('%s' using 22:23:24 title "North center", )", filename );
		printf( R"('%s' using 25:26:27 title "South center", )", filename );
		printf( R"('%s' using 28:29:30 title "South center with offset", )" "\n", filename );

		cout << "angle,   ne.x,   ne.y,   nw.x,   nw.y,   sc.x,   sc.y, sc.err,   so,x,   so,y, so.err,   nc,x,   nc,y" << endl;
	}

	int	i = 0;
	error_south = 0;
#if 0
	point3d_t	pivot_ne {point_ne0.x, point_ne0.y, 0.0};
	point3d_t	pivot_nw {point_nw0.x, point_nw0.y, 0.0};
#endif
	for( double angle = start; angle < end; angle += step ){
		point3d_t	upright_ne, upright_nw, upright_nc, upright_sc, upright_so;
		double	err_so, err_sc;

		upright_ne.x = points_ne[i].x; upright_ne.y = points_ne[i].y; upright_ne.z = 0.0;
		upright_nw.x = points_nw[i].x; upright_nw.y = points_nw[i].y; upright_nw.z = 0.0;
		upright_sc.x = points_sc[i].x; upright_sc.y = points_sc[i].y; upright_sc.z = zs;
		upright_so.x = points_so[i].x; upright_so.y = points_so[i].y; upright_so.z = zs;
		upright_nc.x = points_nc[i].x; upright_nc.y = points_nc[i].y; upright_nc.z = 0.0;
#if 0
		point3d_t	slide_so, normal_original, normal_slide;
		slide_so.x = points_so[i].x; slide_so.y = point_so0.y; slide_so.z = zs;
		normal_vector( normal_original, pivot_ne, pivot_nw, upright_so );
		normal_vector( normal_slide, pivot_ne, pivot_nw, slide_so );
		err_so = RAD2ARCMIN( acos( dot_product( normal_original, normal_slide ) ) );
#else
		err_sc = hypot( points_sc[i].x - center_sc.x, points_sc[i].y - center_sc.y ) - radius_sc;
		err_so = hypot( points_so[i].x - center_so.x, points_so[i].y - center_so.y ) - radius_so;
#endif

		if( !is_evaluation ){
			// 2D plot
			printf("%5.1f,", RAD2DEG(angle) );
			printf("%7.1f,%7.1f,", points_ne[i].x, points_ne[i].y );
			printf("%7.1f,%7.1f,", points_nw[i].x, points_nw[i].y );
			printf("%7.1f,%7.1f,", points_sc[i].x, points_sc[i].y );
			printf("%7.3f,", err_sc );
			printf("%7.1f,%7.1f,", points_so[i].x, points_so[i].y );
			printf("%7.3f,", err_so );
			printf("%7.1f,%7.1f\n", points_nc[i].x, points_nc[i].y );

			// 3D plot
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_ne.x, upright_ne.y, upright_ne.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_nw.x, upright_nw.y, upright_nw.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_sc.x, upright_sc.y, upright_sc.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_so.x, upright_so.y, upright_so.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_nc.x, upright_nc.y, upright_nc.z );

			// 3D plot
			transform( upright_ne, upright_matrix, upright_ne );
			transform( upright_nw, upright_matrix, upright_nw );
			transform( upright_sc, upright_matrix, upright_sc );
			transform( upright_so, upright_matrix, upright_so );
			transform( upright_nc, upright_matrix, upright_nc );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_ne.x, upright_ne.y, upright_ne.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_nw.x, upright_nw.y, upright_nw.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_sc.x, upright_sc.y, upright_sc.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f, ", upright_so.x, upright_so.y, upright_so.z );
			fprintf(datafile, "%8.3f, %8.3f, %8.3f\n", upright_nc.x, upright_nc.y, upright_nc.z );
		}

		if( error_south < abs(err_so) ){
			error_south = err_so;
		}

		i++;
	}

	if( !is_evaluation ){
		fclose(datafile);

		vector<point2d_t>::iterator	itr_first = points_ne.begin();
		vector<point2d_t>::reverse_iterator	itr_last = points_ne.rbegin();
		printf("North rail length = %7.3f", hypot( (*itr_first).x - (*itr_last).x, (*itr_first).y - (*itr_last).y ) );
		printf(", up = %7.3f", hypot( (*itr_first).x - point_ne0.x, (*itr_first).y - point_ne0.y ) );
		printf(", down = %7.3f\n", hypot( (*itr_last).x - point_ne0.x, (*itr_last).y - point_ne0.y ) );

		itr_first = points_nc.begin();
		itr_last = points_nc.rbegin();
		printf("North center x-amplitude = %7.3f", abs( (*itr_first).x - (*itr_last).x ) );
		printf(", y-amplitude = %7.3f\n", abs( 0.5*((*itr_first).y + (*itr_last).y) - point_nc0.y  ) );

		itr_first = points_sc.begin();
		itr_last = points_sc.rbegin();
		printf("South center x-amplitude = %7.3f", abs( (*itr_first).x - (*itr_last).x ) );
		printf(", y-amplitude = %7.3f\n", abs( 0.5*((*itr_first).y + (*itr_last).y) - point_sc0.y  ) );

		itr_first = points_so.begin();
		itr_last = points_so.rbegin();
		printf("South center with offset x-amplitude = %7.3f", abs( (*itr_first).x - (*itr_last).x ) );
		printf(", y-amplitude = %7.3f\n", abs( 0.5*((*itr_first).y + (*itr_last).y) - point_so0.y  ) );
	}
}

int main(int argc, char *argv[])
{
	double	length_north = 500.0;		/* mm */
	double	length_south = 500.0;		/* mm */
	double	offset_south = 0.0;		/* mm */
	double	latitude = DEG2RAD(35.0);
	double	distance_ns = 500.0;	/* mm */
	double	inclination_north = DEG2RAD(30.0);
	double	swing = DEG2RAD(8.0);		/* degree */
	double	step = swing/8.0;
	int		opt;
	char	*endptr;
	double	x;
	double	error_south;
	double	inclination_south;
	bool	is_evaluation = false;
	bool	is_triangle = false;

	while( (opt = getopt(argc, argv, "n:s:o:d:l:i:p:het") ) != -1 ){
		switch( opt ){
			case 'n':	/* distance between north rollers	*/
				x = strtod( optarg, &endptr );
				if( *endptr == '\0' ){
					length_north = x;
				}
				break;

			case 'o':	/* depth of south center	*/
				x = strtod( optarg, &endptr );
				if( *endptr == '\0' ){
					offset_south = x;
				}
				break;

			case 's':	/* distance between south rollers	*/
				x = strtod( optarg, &endptr );
				if( *endptr == '\0' ){
					length_south = x;
				}
				break;

			case 'l':	/* latitude	*/
				x = strtod( optarg, &endptr );
				if( *endptr == '\0' ){
					latitude = DEG2RAD(x);
				}
				break;

			case 'i':	/* inclination of north rail */
				x = strtod( optarg, &endptr );
				if( *endptr == '\0' ){
					inclination_north = DEG2RAD(x);
				}
				break;

			case 'd':	/* distance between north and south rail */
				x = strtod( optarg, &endptr );
				if( *endptr == '\0' ){
					distance_ns = x;
				}
				break;

			case 'p':	/* swing */
				x = strtod( optarg, &endptr );
				if( *endptr == '\0' ){
					swing = DEG2RAD(x);
					step = swing/8.0;
				}
				break;

			case 't':
				is_triangle = true;
				break;

			case 'e':
				is_evaluation = true;
				break;

			case 'h':
			default:
				puts(
R"(	-l		latitude in degree
	-s		distance between south rollers in mm
	-n		distance between north rollers in mm
	-d		distance between north and south rail in mm
	-i		inclination of north rail in degree
	-o		depth of south center
	-p		swing in degree
	-t		triangle
)"
				);
				return 1;
		}
	}

	Circumferetial	c(-swing, swing, step, distance_ns, latitude, is_evaluation);
	if( is_evaluation ){
		if( is_triangle ){
			double	length_north_start = length_north*0.5;
			double	length_north_end = length_north*1.5;
			double	length_north_step = (length_north_end - length_north_start)/16;
			for( double ln = length_north_start; ln < length_north_end; ln += length_north_step ){
				double	offset_south_start = offset_south - 100;
				double	offset_south_end = offset_south + 100;
				double	offset_south_step = (offset_south_end - offset_south_start)/16;
				for( double os = offset_south_start; os < offset_south_end; os += offset_south_step ){
					c.roller_on_rail(error_south, ln, os, inclination_north);
					printf("%8.3f, %8.3f, %8.3f\n", ln, os, error_south );
				}
			}
		} else {
			double	length_north_start = length_north*0.5;
			double	length_north_end = length_north*1.5;
			double	length_north_step = (length_north_end - length_north_start)/16;
			length_north_end += length_north_step*0.5;
			for( double ln = length_north_start; ln < length_north_end; ln += length_north_step ){
				double	length_south_start = length_south*0.5;
				double	length_south_end = length_south*1.5;
				double	length_south_step = (length_south_end - length_south_start)/16;
				length_south_end += length_south_step*0.5;
				for( double ls = length_south_start; ls < length_south_end; ls += length_south_step ){
					c.roller_on_rail(error_south, inclination_south, ln, ls, offset_south, inclination_north);
					printf("%8.3f, %8.3f, %8.3f, %8.3f\n", ln, ls, error_south, RAD2DEG(inclination_south) );
				}
			}
		}
	} else {
		if( is_triangle ){
			c.roller_on_rail(error_south, length_north, offset_south, inclination_north);
			c.rail_on_roller(error_south, length_north, offset_south, inclination_north);
		} else {
			c.roller_on_rail(error_south, inclination_south, length_north, length_south, offset_south, inclination_north);
			c.rail_on_roller(error_south, inclination_south, length_north, length_south, offset_south, inclination_north);
		}
	}

	return 0;
}

