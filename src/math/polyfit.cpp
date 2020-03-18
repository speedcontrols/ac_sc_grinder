static float powf_simple(float x, int y)
{
    if (y == 0) return 1;
    if (y == 1) return x;

    float result = x;
    y--;

    for (; y > 0; y--) result *= x;

    return result;
}

// Calculate approximaton polynomial coefficients
// by Ordinary Least Squares method.
//
// order - max power of polynomial element
// x - array x-values
// y - array y-values
// len - number of input points (x & y length)
// result - array of calculated (order + 1) coefficients (result)
//
void polyfit(int order, float x[], float y[], int len, float result[])
{
    // Array for sums of powers of x values,
    // used in approximation algorithm
    float sums_of_x_powers[2 * order + 1];
    // Array for x^i*y sums values,
    // used in approximation algorithm
    float sums_of_xy_powers[order + 1];
    // Equations matrix for approximation
    float equations_matrix[order + 1][order + 2];

    for (int i = 0; i < 2 * order + 1; i++)
    {
        sums_of_x_powers[i] = 0;

        // Calculate sums of powers of x values
        for (int j = 0; j < len; j++)
        {
            sums_of_x_powers[i] += powf_simple(x[j], i);
        }
    }

    for (int i = 0; i <= order; i++)
    {
        // Build equations matrix from sums of powers of x values
        for (int j = 0;j <= order; j++)
        {
            equations_matrix[i][j] = sums_of_x_powers[i + j];
        }
    }

    for (int i = 0; i < order + 1; i++)
    {
        sums_of_xy_powers[i] = 0;
        // Calculate x^i*y sums
        for (int j=0; j < len; j++)
        {
            sums_of_xy_powers[i] += powf_simple(x[j], i) * y[j];
        }
    }
    // Add x^i*y sums to equations matrix
    for (int i = 0; i <= order; i++)
    {
        equations_matrix[i][order + 1] = sums_of_xy_powers[i];
    }
    // Convert equations matrix to triangular form
    for (int i = 0; i < order; i++)
    {
        for (int k = i + 1; k < order + 1; k++)
        {
            float t = equations_matrix[k][i] / equations_matrix[i][i];
            for (int j = 0; j <= order + 1; j++)
            {
                // Eliminate elements below pivot
                equations_matrix[k][j] = equations_matrix[k][j] - t * equations_matrix[i][j];
            }
        }
    }
    // Calculate solution values of equations
    // This values is approximation polynomial coefficients
    for (int i = order; i >= 0; i--)
    {
        // Initialize solution value with right-hand side
        // of equation
        result[i] = equations_matrix[i][order + 1];

        // Back-substitution: substract previously
        // calculated solution values
        // multiplied by equation coefficients
        for (int j = i + 1; j < order + 1; j++)
        {
            result[i] -= equations_matrix[i][j] * result[j];
        }
        // Normalize solution value
        result[i] = result[i] / equations_matrix[i][i];
    }
}
