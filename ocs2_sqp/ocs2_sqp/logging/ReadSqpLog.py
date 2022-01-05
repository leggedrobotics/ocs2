import pandas as pd
import matplotlib.pyplot as plt
import itertools

if __name__ == "__main__":
    # ===== Settings =======
    fileName = "/tmp/ocs2/sqp_log/log_Tue_Jan__4_23:53:32_2022.txt"

    lineWidth = 1.0
    # ======================

    # Read the log
    data = pd.read_csv(fileName, sep=r'\s*,\s*',)
    data['global_iteration'] = data.index
    firstIterations = data.groupby('problemNumber').first().reset_index()
    lastIterations = data.groupby('problemNumber').last().reset_index()

    # Detect if we have multiple iterations per problem
    multiIterationPlots = len(data) > len(firstIterations)

    # plot iterations
    if multiIterationPlots:
        fig1, ax1 = plt.subplots()
        ax1.plot(lastIterations['problemNumber'], 1 + lastIterations['iteration'], linewidth=lineWidth)
        plt.ylabel('Number of iterations')
        plt.xlabel('Problem number')

    # plot constraint satisfaction per problem
    fig1, ax1 = plt.subplots()
    ax1.semilogy(firstIterations['problemNumber'], firstIterations['totalConstraintViolationBaseline'], linewidth=lineWidth, label='initialization')
    ax1.semilogy(lastIterations['problemNumber'], lastIterations['totalConstraintViolationAfterStep'], linewidth=lineWidth, label='optimized')
    plt.ylabel('Constraint violation norm')
    plt.xlabel('Problem number')
    ax1.legend()

    if multiIterationPlots:
        # plot constraint satisfaction per iteration
        fig1, ax1 = plt.subplots()
        ax1.semilogy(data['global_iteration'], data['totalConstraintViolationAfterStep'], linewidth=lineWidth, label='after iteration')
        ax1.scatter(firstIterations['global_iteration'], firstIterations['totalConstraintViolationBaseline'], marker='.', label='initialization')
        ax1.scatter(lastIterations['global_iteration'], lastIterations['totalConstraintViolationAfterStep'], marker='.', label='optimized')
        plt.ylabel('Constraint violation norm')
        plt.xlabel('Iteration')
        ax1.legend()

    # plot Merit satisfaction per problem
    fig1, ax1 = plt.subplots()
    ax1.plot(firstIterations['problemNumber'], firstIterations['baselinePerformanceIndex/merit'], linewidth=lineWidth, label='initialization')
    ax1.plot(lastIterations['problemNumber'], lastIterations['performanceAfterStep/merit'], linewidth=lineWidth, label='optimized')
    plt.ylabel('Merit value')
    plt.xlabel('Problem number')
    ax1.legend()

    if multiIterationPlots:
        # plot Merit satisfaction per iteration
        fig1, ax1 = plt.subplots()
        ax1.plot(data['global_iteration'], data['performanceAfterStep/merit'], linewidth=lineWidth, label='after iteration')
        ax1.scatter(firstIterations['global_iteration'], firstIterations['baselinePerformanceIndex/merit'], marker='.', label='initialization')
        ax1.scatter(lastIterations['global_iteration'], lastIterations['performanceAfterStep/merit'], marker='.', label='optimized')
        plt.ylabel('Merit value')
        plt.xlabel('Iteration')
        ax1.legend()

    # plot primal update
    fig1, ax1 = plt.subplots()
    ax1.semilogy(data['global_iteration'], data['dxNorm'], linewidth=lineWidth, label='|dx(t)|')
    ax1.semilogy(data['global_iteration'], data['duNorm'], linewidth=lineWidth, label='|du(t)|')
    if multiIterationPlots:
        ax1.scatter(firstIterations['global_iteration'], firstIterations['dxNorm'], marker='.', label='|dx(t)| (first iter)')
        ax1.scatter(firstIterations['global_iteration'], firstIterations['duNorm'], marker='.', label='|du(t)| (first iter)')
    plt.ylabel('Primal update norm')
    plt.xlabel('Iteration')
    ax1.legend()

    # plot step size and type
    fig1, ax1 = plt.subplots()
    marker = itertools.cycle(('.', 'v', 'p', 'x', '*'))
    # ax1.plot(data['global_iteration'], data['stepSize'], linewidth=lineWidth, label='step size')
    for name, group in data.groupby('stepType'):
        ax1.scatter(group['global_iteration'], group['stepSize'], marker=next(marker), label=name)
    plt.ylabel('Step size')
    plt.xlabel('Iteration')
    ax1.legend()

    # Termination condition
    if multiIterationPlots:
        fig1, ax1 = plt.subplots()
        ax1.plot(lastIterations['problemNumber'], lastIterations['convergence'], linewidth=lineWidth)
        plt.title('Convergence condition')
        plt.xlabel('Problem number')

    # Computation times
    fig1, ax1 = plt.subplots()
    ax1.plot(data['global_iteration'], data['linearQuadraticApproximationTime'], linewidth=lineWidth, label='LQ approximation')
    ax1.plot(data['global_iteration'], data['solveQpTime'], linewidth=lineWidth, label='QP solve')
    ax1.plot(data['global_iteration'], data['linesearchTime'], linewidth=lineWidth, label='Linesearch')
    plt.ylabel('CPU time [ms]')
    plt.xlabel('Problem number')
    ax1.legend()

    plt.show()






