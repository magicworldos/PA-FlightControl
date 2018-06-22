/*
 * matrix.c
 *
 *  Created on: Aug 25, 2016
 *      Author: lidq
 */

#include "matrix.h"

//初始化矩阵
int matrix_init(s_Matrix *matrix, int m, int n)
{
	if (matrix == NULL)
	{
		return -1;
	}

	if (m <= 0 || n <= 0)
	{
		return -1;
	}

	//初始化矩阵大小
	matrix->m = m;
	matrix->n = n;

	//申请存放矩阵数据的内存空间
	matrix->v = (num *) malloc(sizeof(num) * matrix->m * matrix->n);
	if (matrix->v == NULL)
	{
		return -1;
	}

	//将矩阵中所有的元素置为0
	matrix_zero(matrix);

	return 0;
}

//销毁矩阵
int matrix_destory(s_Matrix *matrix)
{
	if (matrix == NULL)
	{
		return -1;
	}

	//释放矩阵元素占用的内存空间
	if (matrix->v != NULL)
	{
		free(matrix->v);
	}

	//清除矩阵大小
	matrix->m = 0;
	matrix->n = 0;

	return 0;
}

//置空矩阵
int matrix_zero(s_Matrix *matrix)
{
	if (matrix == NULL)
	{
		return -1;
	}

	if (matrix->v == NULL)
	{
		return -1;
	}

	if (matrix->m <= 0 || matrix->n <= 0)
	{
		return -1;
	}

	//将所有元素的值置为0
	for (int i = 0; i < matrix->m; i++)
	{
		for (int j = 0; j < matrix->n; j++)
		{
			matrix->v[i * matrix->n + j] = 0;
		}
	}

	return 0;
}

//矩阵相加
int matrix_add(s_Matrix *result, s_Matrix *src, s_Matrix *src1)
{
	if (result == NULL)
	{
		return -1;
	}

	if (result->v == NULL)
	{
		return -1;
	}

	if (src == NULL)
	{
		return -1;
	}

	if (src->v == NULL)
	{
		return -1;
	}

	if (src->m <= 0 || src->n <= 0)
	{
		return -1;
	}

	if (src1 == NULL)
	{
		return -1;
	}

	if (src1->v == NULL)
	{
		return -1;
	}

	if (src1->m <= 0 || src1->n <= 0)
	{
		return -1;
	}

	//矩阵加法行列数必须相等
	if (src->m != src1->m || src->n != src1->n)
	{
		return -1;
	}

	//循环数组
	for (int i = 0; i < src->m; i++)
	{
		for (int j = 0; j < src->n; j++)
		{
			//元素相加，结果存入result矩阵中
			result->v[i * src->n + j] = src->v[i * src->n + j] + src1->v[i * src->n + j];
		}
	}

	//结果矩阵与相加矩阵大小相等
	result->m = src->m;
	result->n = src->n;

	return 0;
}

//矩阵加上一个数
int matrix_add_num(s_Matrix *result, s_Matrix *src, num v)
{
	if (result == NULL)
	{
		return -1;
	}

	if (result->v == NULL)
	{
		return -1;
	}

	if (src == NULL)
	{
		return -1;
	}

	if (src->v == NULL)
	{
		return -1;
	}

	if (src->m <= 0 || src->n <= 0)
	{
		return -1;
	}

	//每一个元素都加上这个数
	for (int i = 0; i < src->m; i++)
	{
		for (int j = 0; j < src->n; j++)
		{
			result->v[i * src->n + j] = src->v[i * src->n + j] + v;
		}
	}

	result->m = src->m;
	result->n = src->n;

	return 0;
}

int matrix_add_num_slef(s_Matrix *self, num v)
{

	if (self == NULL)
	{
		return -1;
	}

	if (self->v == NULL)
	{
		return -1;
	}

	if (self->m <= 0 || self->n <= 0)
	{
		return -1;
	}

	for (int i = 0; i < self->m; i++)
	{
		for (int j = 0; j < self->n; j++)
		{
			self->v[i * self->n + j] += v;
		}
	}

	return 0;
}

//矩阵相乘
int matrix_mult(s_Matrix *result, s_Matrix *src, s_Matrix *src1)
{
	if (result == NULL)
	{
		return -1;
	}

	if (result->v == NULL)
	{
		return -1;
	}

	if (src == NULL)
	{
		return -1;
	}

	if (src->v == NULL)
	{
		return -1;
	}

	if (src->m <= 0 || src->n <= 0)
	{
		return -1;
	}

	if (src1 == NULL)
	{
		return -1;
	}

	if (src1->v == NULL)
	{
		return -1;
	}

	if (src1->m <= 0 || src1->n <= 0)
	{
		return -1;
	}

	//只有当第一个矩阵（左侧矩阵）的列数等于第二个矩阵（右侧矩阵）的行数时，两个矩阵才能相乘
	if (src->n != src1->m)
	{
		return -1;
	}

	//结果矩阵的行数为第一个矩阵的行数
	result->m = src->m;
	//结果矩阵的列数为第二个矩阵的列数
	result->n = src1->n;

	//循环每行
	for (int i = 0; i < result->m; i++)
	{
		//循环每列
		for (int j = 0; j < result->n; j++)
		{
			//计算第一个矩阵与第二个矩阵的乘法结果的累加和
			num v = 0;
			for (int k = 0; k < src->n; k++)
			{
				//乘法结果的累加和
				v += src->v[i * src->n + k] * src1->v[k * src1->n + j];
			}
			//得到结果
			result->v[i * result->n + j] = v;
		}
	}

	return 0;
}

//转置矩阵
int matrix_transposition(s_Matrix *result, s_Matrix *src)
{
	if (result == NULL)
	{
		return -1;
	}

	if (result->v == NULL)
	{
		return -1;
	}

	if (src == NULL)
	{
		return -1;
	}

	if (src->v == NULL)
	{
		return -1;
	}

	if (src->m <= 0 || src->n <= 0)
	{
		return -1;
	}

	//行数等于原列数
	result->m = src->n;
	//列数等于原行数
	result->n = src->m;

	//循环所有元素
	for (int i = 0; i < src->m; i++)
	{
		for (int j = 0; j < src->n; j++)
		{
			//列号变列号，列号变行号
			result->v[j * result->n + i] = src->v[i * src->n + j];
		}
	}

	return 0;
}

int matrix_mult_num(s_Matrix *result, s_Matrix *src, num v)
{
	if (result == NULL)
	{
		return -1;
	}

	if (result->v == NULL)
	{
		return -1;
	}

	if (src == NULL)
	{
		return -1;
	}

	if (src->v == NULL)
	{
		return -1;
	}

	if (src->m <= 0 || src->n <= 0)
	{
		return -1;
	}

	result->m = src->m;
	result->n = src->n;

	for (int i = 0; i < src->m; i++)
	{
		for (int j = 0; j < src->n; j++)
		{
			result->v[i * result->n + j] = src->v[i * src->n + j] * v;
		}
	}

	return 0;
}

int matrix_mult_num_self(s_Matrix *self, num v)
{
	if (self == NULL)
	{
		return -1;
	}

	if (self->v == NULL)
	{
		return -1;
	}

	if (self->m <= 0 || self->n <= 0)
	{
		return -1;
	}

	for (int i = 0; i < self->m; i++)
	{
		for (int j = 0; j < self->n; j++)
		{
			self->v[i * self->n + j] *= v;
		}
	}

	return 0;
}

//计算行列式
int matrix_determinant(num *result, s_Matrix *src)
{
	if (src == NULL)
	{
		return -1;
	}

	if (src->v == NULL)
	{
		return -1;
	}

	if (src->m <= 0 || src->n <= 0)
	{
		return -1;
	}

	//行列式的行数与列数必须相同
	if (src->m != src->n)
	{
		return -1;
	}

	if (result == NULL)
	{
		return -1;
	}

	if (src->m == 1)
	{
		*result = src->v[0];
		return 0;
	}

	//如果是2阶行列式
	if (src->m == 2)
	{
		//直接计算
		*result = src->v[0] * src->v[3] - src->v[1] * src->v[2];
		return 0;
	}

	num s = 0;
	//计算其中一行
	for (int k = 0; k < src->m; k++)
	{
		num det = 0;
		s_Matrix cofactor;
		matrix_init(&cofactor, src->m - 1, src->n - 1);
		//计算代数余子式
		for (int i = 1, ci = 0; i < src->m; i++, ci++)
		{
			for (int j = 0, cj = 0; j < src->n; j++)
			{
				if (j != k)
				{
					cofactor.v[ci * cofactor.n + cj] = src->v[i * src->n + j];
					cj++;
				}
			}
		}
		//递归计算代数余子式
		matrix_determinant(&det, &cofactor);
		matrix_destory(&cofactor);
		//当前元素乘以其代数余子式的累加和
		s += src->v[k] * pow(-1, 0 + k) * det;
	}

	//返回行列式的值
	*result = s;

	return 0;
}

//伴随矩阵
int matrix_adjoint(s_Matrix *result, s_Matrix *src)
{
	if (result == NULL)
	{
		return -1;
	}

	if (result->v == NULL)
	{
		return -1;
	}

	if (result->m <= 0 || result->n <= 0)
	{
		return -1;
	}

	if (src == NULL)
	{
		return -1;
	}

	if (src->v == NULL)
	{
		return -1;
	}

	if (src->m <= 0 || src->n <= 0)
	{
		return -1;
	}

	//必须是方阵
	if (src->m != src->n)
	{
		return -1;
	}

	result->m = src->m;
	result->n = src->n;

	if (src->m == 1)
	{
		result->v[0] = 1;
		return 0;
	}

	int cm = src->m - 1;
	int cn = src->n - 1;

	s_Matrix cofactor;
	matrix_init(&cofactor, cm, cn);

	//计算每一个元素的代数余子式并作为这个元素所在位置的值
	for (int i = 0; i < src->m; i++)
	{
		for (int j = 0; j < src->n; j++)
		{
			for (int k = 0, ck = 0; k < src->m; k++)
			{
				//抹去第i行
				if (k != i)
				{
					for (int l = 0, cl = 0; l < src->n; l++)
					{
						//抹去第j列
						if (l != j)
						{
							//计算值
							cofactor.v[ck * cofactor.n + cl] = src->v[k * src->n + l];
							cl++;
						}
					}
					ck++;
				}
			}
			num det = 0;
			//计算代数余子式
			matrix_determinant(&det, &cofactor);
			//用代数余子式做为新元素
			result->v[j * result->n + i] = pow(-1, i + j) * det;
		}
	}

	matrix_destory(&cofactor);

	return 0;
}

int matrix_inv(s_Matrix *result, s_Matrix *src)
{
	if (result == NULL)
	{
		return -1;
	}

	if (result->v == NULL)
	{
		return -1;
	}

	if (result->m <= 0 || result->n <= 0)
	{
		return -1;
	}

	if (src == NULL)
	{
		return -1;
	}

	if (src->v == NULL)
	{
		return -1;
	}

	if (src->m <= 0 || src->n <= 0)
	{
		return -1;
	}

	if (src->m != src->n)
	{
		return -1;
	}

	num det = 0;
	matrix_determinant(&det, src);
	if (fabs(det) < DBL_EPSILON)
	{
		return -1;
	}

	s_Matrix adjoint;
	matrix_init(&adjoint, src->m, src->n);
	matrix_adjoint(&adjoint, src);
	matrix_mult_num(result, &adjoint, 1.0 / det);

	matrix_destory(&adjoint);

	return 0;
}

//逆矩阵
int matrix_inverse(s_Matrix *result, s_Matrix *src)
{
	if (src == NULL)
	{
		return -1;
	}

	if (src->v == NULL)
	{
		return -1;
	}

	if (src->m <= 0 || src->n <= 0)
	{
		return -1;
	}

	if (src->m != src->n)
	{
		return -1;
	}

	if (result == NULL)
	{
		return -1;
	}

	if (result->v == NULL)
	{
		return -1;
	}

	if (result->m <= 0 || result->n <= 0)
	{
		return -1;
	}

	//必须是方阵
	if (result->m != src->n)
	{
		return -1;
	}

	//一阶方阵
	if (src->m == 1)
	{
		result->v[0] = 1.0 / src->v[0];
		return 0;
	}

	int err = 0;
	int n = src->m;
	s_Matrix temp;
	matrix_init(&temp, n, n * 2);
	//(A, E)，矩阵大小由m，n变为m，2n
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			temp.v[i * (n * 2) + j] = src->v[i * n + j];
		}
		temp.v[i * (n * 2) + (i + n)] = 1;
	}

	//对首行做初等变换
	for (int i = 0; i < n; i++)
	{
		for (int row = 0; row < n; row++)
		{
			//将当前对角线上的元素值保留，其它行上的值为0
			if (row != i)
			{
				if (fabs(temp.v[i * (n * 2) + i]) < DBL_EPSILON)
				{
					err = 1;
					goto _label_inf;
				}
				//计算初等变换参数
				num a = -temp.v[row * (n * 2) + i] / temp.v[i * (n * 2) + i];
				//初等变换
				for (int j = i; j < (n * 2); j++)
				{
					temp.v[row * (n * 2) + j] += temp.v[i * (n * 2) + j] * a;
				}
			}
		}
	}

	//除以对角线的值，将当前对角线上的元素变为1
	for (int i = 0; i < n; i++)
	{
		num a = temp.v[i * (n * 2) + i];
		if (fabs(a) < DBL_EPSILON)
		{
			err = 1;
			goto _label_inf;
		}
		//将当前对角线上的元素变为1
		for (int j = i; j < (n * 2); j++)
		{
			temp.v[i * (n * 2) + j] /= a;
		}
	}

	//取得逆矩阵
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			result->v[i * n + j] = temp.v[i * (n * 2) + (j + n)];
		}
	}

	_label_inf: ;
	matrix_destory(&temp);

	//如果初等变换失败
	if (err)
	{
		//采用伴随矩阵法递归计算
		return matrix_inv(result, src);
	}

	return 0;
}

//显示矩阵内容
int matrix_display(s_Matrix *matrix)
{
	if (matrix == NULL)
	{
		return -1;
	}

	if (matrix->v == NULL)
	{
		return -1;
	}

	if (matrix->m <= 0 || matrix->n <= 0)
	{
		return -1;
	}

	//显示所有元素值
	for (int i = 0; i < matrix->m; i++)
	{
		for (int j = 0; j < matrix->n; j++)
		{
//			printf("%+e\t\t", matrix->v[i * matrix->n + j]);
			printf("%.10f\t\t", matrix->v[i * matrix->n + j]);
		}
		printf("\n");
	}
	printf("\n");

	return 0;
}
