import matplotlib.animation as anim
import matplotlib.pyplot as plt
from cycler import cycler


def plot_2d_traj(subscribers, interval=100):
    fig, ax = plt.subplots()
    fig.patch.set_facecolor("white")

    # Set up color cycle for multiple trajectories
    colors = ["blue", "red", "orange", "purple", "green", "brown"]
    ax.set_prop_cycle(cycler("color", colors))

    def update_plot(_):
        ax.clear()
        ax.set_prop_cycle(cycler("color", colors))
        ax.set_aspect("equal", "box")
        ax.grid(True, linestyle="-.", alpha=0.3)
        _mx = []
        _my = []

        for subscriber in subscribers:
            x, y = subscriber.get_trajectory_data()
            if len(x) > 0 and len(y) > 0:
                ax.plot(x, y, label=subscriber.model.node_name, alpha=0.8)
                _mx.append(max(max(x), max(y)))
                _my.append(min(min(x), min(y)))

        max_val = max(_mx) if _mx else 0
        min_val = min(_my) if _my else 0

        ax.set_xlabel("x [m]", fontsize=12)
        ax.set_ylabel("y [m]", fontsize=12)
        ax.set_title("2D Trajectory Comparison", fontsize=14, fontweight="bold")
        ax.legend(loc="best", fontsize=11)
        axes = plt.gca()
        axes.set_xlim(min_val - 1, max_val + 1)
        axes.set_ylim(min_val - 1, max_val + 1)

        return (ax,)

    ani = anim.FuncAnimation(fig, update_plot, interval=interval, blit=False)
    plt.show()
