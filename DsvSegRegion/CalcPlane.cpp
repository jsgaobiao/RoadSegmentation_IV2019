
#include <stdio.h>
#include <string.h>
#include <cstdio>
#include <fcntl.h>
#include <memory.h>
#include <math.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>


void Find_Scatter_Matrix(double Centroid[3], int Points_Total, 
						 double *X_Coord, double *Y_Coord, double *Z_Coord,
						 double Scatter_Matrix[3][3], int Order[3]);
void tred2(double a[3][3], double d[3], double e[3]);
void tqli(double d[3], double e[3], double z[3][3]);

/*
**	This routine calculates and returns the equation of a plane.
**	It takes as inputs the x,y and z coordinates of a set of points
**	(contained in "X_Coord", "Y_Coord" and "Z_Coord"), an integer
**	"Points_Total" indicating the total number of points, and an integer
**	"Origin_Flag" indicating whether the plane being found should be
**	forced to pass through the origin (0,0,0) or not.  It returns
**	the four variables a,b,c and d (in "Plane_Eq") indicating the
**	equation of the best-fit plane to the set of points.  (aX+bY+cZ+d = 0)
*/
void Calculate_Plane(int Points_Total, double *X_Coord, double *Y_Coord, double *Z_Coord,
					 int Origin_Flag, double Plane_Eq[4])
{
	double	Scatter_Matrix[3][3];
	int		Order[3];
	double	Centroid[3];
	double	Normal[3];
	double	Diagonal_Matrix[3];
	double	Off_Diagonal_Matrix[3];
	double	min,xtot,ytot,ztot;
	int		min_index;
	int		i;

	/*	The first thing to do is to find the center of gravity (Centroid)
	**	of the set of points.  Sometimes it is desired to force the plane
	**	being found to pass through the origin (0,0,0).  If this is the
	**	case, then the Centroid is set equal to (0,0,0).  Note that this is
	**	NOT the true Centroid of the set of points!! It merely makes these
	**	routines and equations easier.
	*/
	if (Origin_Flag == 1) {
		Centroid[0]=0.0;
		Centroid[1]=0.0;
		Centroid[2]=0.0;
	}
	else {
		xtot=ytot=ztot=0.0;
		for (i=0; i<Points_Total; i++) {
			xtot+=X_Coord[i];
			ytot+=Y_Coord[i];
			ztot+=Z_Coord[i];
		}
		Centroid[0]=xtot/(double)Points_Total;
		Centroid[1]=ytot/(double)Points_Total;
		Centroid[2]=ztot/(double)Points_Total;
	}
	/*	The next thing to do is to compute the scatter matrix for the
	**	set of points.
	*/
	Find_Scatter_Matrix(Centroid,Points_Total,X_Coord,Y_Coord,Z_Coord,
			Scatter_Matrix,Order);

	/*	These next two procedures calculate the eigenvectors from the
	**	Scatter_Matrix, from which the equation of the plane is found.
	**	Both these routines were coded by Penny.
	*/
	tred2(Scatter_Matrix,Diagonal_Matrix,Off_Diagonal_Matrix);
	tqli(Diagonal_Matrix,Off_Diagonal_Matrix,Scatter_Matrix);
	/*
	**	Find the smallest eigenvalue first.
	*/
	min=Diagonal_Matrix[0];
	min_index=0;
	for (i=1; i<3; i++)
		if (Diagonal_Matrix[i] < min) {
			min=Diagonal_Matrix[i];
			min_index=i;
		}
	/*
	**	The normal of the plane is the smallest eigenvector.
	*/
	for(i=0;i<3;i++)
		Normal[Order[i]]=Scatter_Matrix[i][min_index];
	/*
	**	Force z-component of the normal to be positive, and copy the
	**	normal into the first three variables of the plane equation.
	*/
	for(i=0;i<3;i++)
		if (Normal[2] < 0.0)
			Plane_Eq[i]=Normal[i]*(-1.0);
	else
		Plane_Eq[i]=Normal[i];
	/*
	**	The equation of a plane is of the form aX+bY+cZ+d=0.  The normal
	**	to the plane has already been found and equates to (a,b,c).  The
	**	fourth variable, "d", can be found by solving the plane equation
	**	with the coordinates of a point (the Centroid) in place of (X,Y,Z).
	**	If the plane is to be forced to pass through the origin (0,0,0),
	**	then the Centroid was earlier set equal to (0,0,0).  This, of
	**	course, is NOT the true Centroid of the set of points!  However,
	**	it will force "d" equal to zero in the following equation, which
	**	means that the plane will pass through the origin (0,0,0).
	*/
	Plane_Eq[3]=-Plane_Eq[0]*Centroid[0]-Plane_Eq[1]*Centroid[1]-
	Plane_Eq[2]*Centroid[2];

}

/*	This routine finds the scatter matrix of a number of points equal
**	to "Points_Total".  The x,y and z coordinates of the points are
**	stored in the "X_Coord", "Y_Coord" and "Z_Coord" arrays.  "Centroid"
**	is a 3-element array containing the center of gravity of the set
**	of points.  The scatter matrix will be returned in the 3x3 array
**	called "Scatter_Matrix".  The xyz placement in the Scatter Matrix
**	will be returned by the 3 element array "Order", where the index
**	of Order indicates the column (and row) in "Scatter_Matrix", and
**	a value of 0 means x, 1 means y, and 2 means z.
*/

void Find_Scatter_Matrix(double Centroid[3], int Points_Total, 
						 double *X_Coord, double *Y_Coord, double *Z_Coord,
						 double Scatter_Matrix[3][3], int Order[3])
{
	int	i,TempI;
	double	TempD;

	/*	To compute the correct scatter matrix, the centroid must be
	**	subtracted from all points.  If the plane is to be forced to pass
	**	through the origin (0,0,0), then the Centroid was earlier set
	**	equal to (0,0,0).  This, of course, is NOT the true Centroid of
	**	the set of points!  Since the matrix is symmetrical about its
	**	diagonal, one-third of it is redundant and is simply found at
	**	the end.
	*/
	for (i=0; i<3; i++)
		Scatter_Matrix[i][0]=Scatter_Matrix[i][1]=Scatter_Matrix[i][2]=0;

	for (i=0; i<Points_Total; i++) {
		Scatter_Matrix[0][0]+=(X_Coord[i]-Centroid[0])*(X_Coord[i]-Centroid[0]);
		Scatter_Matrix[0][1]+=(X_Coord[i]-Centroid[0])*(Y_Coord[i]-Centroid[1]);
		Scatter_Matrix[0][2]+=(X_Coord[i]-Centroid[0])*(Z_Coord[i]-Centroid[2]);
		Scatter_Matrix[1][1]+=(Y_Coord[i]-Centroid[1])*(Y_Coord[i]-Centroid[1]);
		Scatter_Matrix[1][2]+=(Y_Coord[i]-Centroid[1])*(Z_Coord[i]-Centroid[2]);
		Scatter_Matrix[2][2]+=(Z_Coord[i]-Centroid[2])*(Z_Coord[i]-Centroid[2]);
	}

	Scatter_Matrix[1][0]=Scatter_Matrix[0][1];
	Scatter_Matrix[2][0]=Scatter_Matrix[0][2];
	Scatter_Matrix[2][1]=Scatter_Matrix[1][2];
	/*	Now, perform a sort of "Matrix-sort", whereby all the larger elements
	**	in the matrix are relocated towards the lower-right portion of the
	**	matrix.  This is done as a requisite of the tred2 and tqli algorithms,
	**	for which the scatter matrix is being computed as an input.
	**	"Order" is a 3 element array that will keep track of the xyz order
	**	in the Scatter_Matrix.
	*/
	Order[0]=0;		/* Beginning order is x-y-z, as found above */
	Order[1]=1;
	Order[2]=2;
	if (Scatter_Matrix[0][0] > Scatter_Matrix[1][1]) {
		TempD=Scatter_Matrix[0][0];
		Scatter_Matrix[0][0]=Scatter_Matrix[1][1];
		Scatter_Matrix[1][1]=TempD;
		TempD=Scatter_Matrix[0][2];
		Scatter_Matrix[0][2]=Scatter_Matrix[2][0]=Scatter_Matrix[1][2];
		Scatter_Matrix[1][2]=Scatter_Matrix[2][1]=TempD;
		TempI=Order[0];
		Order[0]=Order[1];
		Order[1]=TempI;
	}

	if (Scatter_Matrix[1][1] > Scatter_Matrix[2][2]) {
		TempD=Scatter_Matrix[1][1];
		Scatter_Matrix[1][1]=Scatter_Matrix[2][2];
		Scatter_Matrix[2][2]=TempD;
		TempD=Scatter_Matrix[0][1];
		Scatter_Matrix[0][1]=Scatter_Matrix[1][0]=Scatter_Matrix[0][2];
		Scatter_Matrix[0][2]=Scatter_Matrix[2][0]=TempD;
		TempI=Order[1];
		Order[1]=Order[2];
		Order[2]=TempI;
	}

	if (Scatter_Matrix[0][0] > Scatter_Matrix[1][1]) {
		TempD=Scatter_Matrix[0][0];
		Scatter_Matrix[0][0]=Scatter_Matrix[1][1];
		Scatter_Matrix[1][1]=TempD;
		TempD=Scatter_Matrix[0][2];
		Scatter_Matrix[0][2]=Scatter_Matrix[2][0]=Scatter_Matrix[1][2];
		Scatter_Matrix[1][2]=Scatter_Matrix[2][1]=TempD;
		TempI=Order[0];
		Order[0]=Order[1];
		Order[1]=TempI;
	}
}

/*
**	This routine calculates the residuals error.  In other words, it
**	sums the squares of the orthogonal distances from the given set
**	of points to the given plane equation.
*/
void Calculate_Residuals(double *X, double *Y, double *Z, double Equation[4], 
						 double *Error, int PointsTotal)

{
	int     i,t;
	double  distance;

	/* "distance" is calculated as the distance between each point */
	/* (X[n],Y[n],Z[n]) and the plane (Equation). */
	/* "Error" is the avg of all these distances. */

	*Error=0.0;
	t=0;
	for (i=0; i<PointsTotal; i++) {
		distance=(Equation[0]*X[i]+Equation[1]*Y[i]+Equation[2]*
		Z[i]+Equation[3])/sqrt(Equation[0]*Equation[0]+Equation[1]*
		Equation[1]+Equation[2]*Equation[2]);
		*Error=*Error+fabs(distance);
		t++;
	}

	(*Error)/=(double)t;
}

/*
**	This code is taken from ``Numerical Recipes in C'', 2nd
**	and 3rd editions, by Press, Teukolsky, Vetterling and
**	Flannery, Cambridge University Press, 1992, 1994.
**
**	In the forward of their book, the authors proclaim that
**	this code may only be used if you obtain a copy of the book.
**	I have a copy.  If you plan on using this code for prolonged
**	stretches, please buy a copy of the book.  Or at least acknowledge
**	this disclaimer.
**
**	-Adam
*/


/*
**	tred2 Householder reduction of a real, symmetric matrix a[1..n][1..n].
**	On output, a is replaced by the orthogonal matrix q effecting the
**	transformation. d[1..n] returns the diagonal elements of the
**	tridiagonal matrix, and e[1..n] the off-diagonal elements, with 
**	e[1]=0.
**
**	For my problem, I only need to handle a 3x3 symmetric matrix,
**	so it can be simplified.
**	Therefore n=3.
**
**	Attention: in the book, the index for array starts from 1,
**	but in C, index should start from zero. so I need to modify it.
**	I think it is very simple to modify, just substract 1 from all the
**	index.
*/

#define	SIGN(a,b)	((b)<0? -fabs(a):fabs(a))

void tred2(double a[3][3], double d[3], double e[3])
{
  int		l,k,i,j;
  double	scale,hh,h,g,f;

	for(i=3;i>=2;i--)
	{
	l=i-1;
	h=scale=0.0;
	if(l>1)
		{
		for(k=1;k<=l;k++)
			scale+=fabs(a[i-1][k-1]);
		if(scale==0.0)		/* skip transformation */
			e[i-1]=a[i-1][l-1];
		else
			{
			for(k=1;k<=l;k++)
				{
				a[i-1][k-1]/=scale;	/* use scaled a's for transformation. */
				h+=a[i-1][k-1]*a[i-1][k-1];	/* form sigma in h. */
				}
			f=a[i-1][l-1];
			g=f>0? -sqrt(h):sqrt(h);
			e[i-1]=scale*g;
			h-=f*g;	/* now h is equation (11.2.4) */
			a[i-1][l-1]=f-g;	/* store u in the ith row of a. */
			f=0.0;
			for(j=1;j<=l;j++)
				{
				a[j-1][i-1]=a[i-1][j-1]/h; /* store u/H in ith column of a. */
				g=0.0;	/* form an element of A.u in g */
				for(k=1;k<=j;k++)
					g+=a[j-1][k-1]*a[i-1][k-1];
				for(k=j+1;k<=l;k++)
					g+=a[k-1][j-1]*a[i-1][k-1];
				e[j-1]=g/h; /* form element of p in temorarliy unused element of e. */
				f+=e[j-1]*a[i-1][j-1];
				}
			hh=f/(h+h);	/* form K, equation (11.2.11) */
			for(j=1;j<=l;j++) /* form q and store in e overwriting p. */
				{
				f=a[i-1][j-1]; /* Note that e[l]=e[i-1] survives */
				e[j-1]=g=e[j-1]-hh*f;
				for(k=1;k<=j;k++) /* reduce a, equation (11.2.13) */
					a[j-1][k-1]-=(f*e[k-1]+g*a[i-1][k-1]);
				}
			}
		}
	else
		e[i-1]=a[i-1][l-1];
	d[i-1]=h;
	}


  /*
  **	For computing eigenvector.
  */
  d[0]=0.0;
  e[0]=0.0;

  for(i=1;i<=3;i++)/* begin accumualting of transfomation matrices */
	{
	l=i-1;
	if(d[i-1]) /* this block skipped when i=1 */
		{
		for(j=1;j<=l;j++)
			{
			g=0.0;
			for(k=1;k<=l;k++) /* use u and u/H stored in a to form P.Q */
				g+=a[i-1][k-1]*a[k-1][j-1];
			for(k=1;k<=l;k++)
				a[k-1][j-1]-=g*a[k-1][i-1];
			}
		}	
	d[i-1]=a[i-1][i-1];
	a[i-1][i-1]=1.0; /* reset row and column of a to identity matrix for next iteration */
	for(j=1;j<=l;j++)
		a[j-1][i-1]=a[i-1][j-1]=0.0;
	}
}



/*
**	QL algo with implicit shift, to determine the eigenvalues and 
**	eigenvectors of a real,symmetric  tridiagonal matrix, or of a real, 
**	symmetric matrix previously reduced by algo tred2.
**	On input , d[1..n] contains the diagonal elements of the tridiagonal
**	matrix. On output, it returns the eigenvalues. The vector e[1..n]
**	inputs the subdiagonal elements of the tridiagonal matrix, with e[1]
**	arbitrary. On output e is destroyed. If the eigenvectors of a 
**	tridiagonal matrix are desired, the matrix z[1..n][1..n] is input
**	as the identity matrix. If the eigenvectors of a matrix that has 
**	been reduced by tred2 are required, then z is input as the matrix 
**	output by tred2. In either case, the kth column of z returns the 
**	normalized eigenvector corresponding to d[k]. 
**
*/
void tqli(double d[3],double e[3],double z[3][3])
{
  int		m,l,iter,i,k;
  double	s,r,p,g,f,dd,c,b;

  for(i=2;i<=3;i++)
	e[i-2]=e[i-1];	/* convenient to renumber the elements of e */
  e[2]=0.0;
  for(l=1;l<=3;l++)
	{
	iter=0;
	do
		{
		for(m=l;m<=2;m++)
			{
			/*
			**	Look for a single small subdiagonal element
			**	to split the matrix.
			*/
			dd=fabs(d[m-1])+fabs(d[m]);
			if(fabs(e[m-1])+dd == dd)
				break;
			}
		if(m!=l)
			{
			if(iter++ == 30)
				{
				printf("\nToo many iterations in TQLI");
				}
			g=(d[l]-d[l-1])/(2.0*e[l-1]); /* form shift */
			r=sqrt((g*g)+1.0);
			g=d[m-1]-d[l-1]+e[l-1]/(g+SIGN(r,g)); /* this is dm-ks */
			s=c=1.0;
			p=0.0;
			for(i=m-1;i>=l;i--)
				{
				/*
				**	A plane rotation as in the original 
				**	QL, followed by Givens rotations to
				**	restore tridiagonal form.
				*/
				f=s*e[i-1];
				b=c*e[i-1];
				if(fabs(f) >= fabs(g))
					{
					c=g/f;
					r=sqrt((c*c)+1.0);
					e[i]=f*r;
					c*=(s=1.0/r);
					}
				else
					{
					s=f/g;
					r=sqrt((s*s)+1.0);
					e[i]=g*r;
					s*=(c=1.0/r);
					}
				g=d[i]-p;
				r=(d[i-1]-g)*s+2.0*c*b;
				p=s*r;
				d[i]=g+p;
				g=c*r-b;
				for(k=1;k<=3;k++)
					{
					/*
					**	Form eigenvectors 
					*/
					f=z[k-1][i];
					z[k-1][i]=s*z[k-1][i-1]+c*f;
					z[k-1][i-1]=c*z[k-1][i-1]-s*f;
					}
				}
			d[l-1]=d[l-1]-p;
			e[l-1]=g;
			e[m-1]=0.0;		
			}
		}while(m != l);
	}
}




