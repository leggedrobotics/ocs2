import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
import itertools

if __name__ == "__main__":
    # ===== Settings =======
    fileName = "/tmp/ocs2/sqp_log/log_Wed_Jan_19_10:02:29_2022.txt"

    lineWidth = 1.0
    # ======================
    rc('text', usetex=True)
    plt.rcParams['text.latex.preamble'].join([
        r'\usepackage{tgheros}',  # helvetica font
        r'\usepackage{sansmath}',  # math-font matching  helvetica
        r'\sansmath'  # actually tell tex to use it!
        r'\usepackage{siunitx}',  # micro symbols
        r'\sisetup{detect-all}',  # force siunitx to use the fonts
    ])

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
    ax1.plot(data['global_iteration'], data['linearQuadraticApproximationTime'] + data['solveQpTime'] + data['linesearchTime'], linewidth=lineWidth, label='Total')
    plt.ylabel('CPU time [ms]')
    plt.xlabel('Problem number')
    ax1.legend()

    # Problem init time
    fig1, ax1 = plt.subplots()
    ax1.plot(firstIterations['global_iteration'], firstIterations['time'], linewidth=lineWidth, marker='.', label='t0')
    plt.ylabel('t0 [s]')
    plt.xlabel('Problem number')
    ax1.legend()

    # Cost - constraints - stepsize when using realtime iteration
    if not multiIterationPlots:
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

        ax1.plot(lastIterations['time'], lastIterations['performanceAfterStep/merit'], linewidth=lineWidth)
        ax1.set_ylabel('Cost')
        ax1.grid(linestyle=':')

        ax2.semilogy(lastIterations['time'], lastIterations['totalConstraintViolationAfterStep'], linewidth=lineWidth)
        ax2.set_ylabel('Constraint violation')
        ax2.set_yticks(np.logspace(-6, 0, num=4))
        ax2.grid(linestyle=':')

        marker = itertools.cycle(('v', '^', 'x', '*'))
        offset = itertools.cycle((0.05, -0.05, 0.0, 0.0))
        # ax1.plot(data['global_iteration'], data['stepSize'], linewidth=lineWidth, label='step size')
        for name, group in data.groupby('stepType'):
            ax3.scatter(group['time'], group['stepSize'] + next(offset), marker=next(marker), label=name, alpha=0.7)
        ax3.set_ylim(0.0, 1.15)
        ax3.set_ylabel('Step size')
        ax3.set_xlabel('time [s]')
        ax3.set_yticks([0.0, 0.25, 0.5, 1.0])
        ax3.legend()
        ax3.grid(linestyle=':')

        fig.align_ylabels()

    plt.show()






