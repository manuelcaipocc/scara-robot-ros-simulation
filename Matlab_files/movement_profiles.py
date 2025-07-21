from ruckig import Ruckig, InputParameter, OutputParameter, Result
import numpy as np
from scipy.io import savemat
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import rcParams
from pathlib import Path
import os

# ===== Professional figure styling =====
mpl.rcParams.update({
    'font.size': 12,
    'font.family': 'serif',
    'axes.labelsize': 14,
    'axes.titlesize': 16,
    'xtick.labelsize': 12,
    'ytick.labelsize': 12,
    'legend.fontsize': 12,
    'figure.titlesize': 18,
    'grid.linestyle': '--',
    'grid.alpha': 0.7,
    'lines.linewidth': 2,
})

# ===== Joint parameters =====
axes = {
    'phi0': {'s': np.deg2rad(150), 'v': 0.5 * np.pi, 'a': 2 * np.pi, 'r': 1200},
    'phi1': {'s': np.deg2rad(90), 'v': 0.5 * np.pi, 'a': np.pi, 'r': 1000},
    'phi2': {'s': np.deg2rad(130), 'v': np.pi, 'a': 3 * np.pi, 'r': 500},
    'dh3': {'s': 0.36, 'v': 100 / 60, 'a': 0.8, 'r': 500},
    'phi4': {'s': np.deg2rad(160), 'v': 2 * np.pi, 'a': 5 * np.pi, 'r': 8000}
}

waypoints = {
    'phi0': [0, axes['phi0']['s'], -axes['phi0']['s'], 0],
    'phi1': [0, axes['phi1']['s'], 0],
    'phi2': [0, axes['phi2']['s'], -axes['phi2']['s'], 0],
    'dh3': [0, axes['dh3']['s']],
    'phi4': [0, axes['phi4']['s'], -axes['phi4']['s'], 0],
}

# ===== Simulation configuration =====
time_step = 0.01
otg = Ruckig(1, time_step)
output_folder = Path("plots")
output_folder.mkdir(exist_ok=True)

# ===== Function to prepare InputParameter =====
def create_input(current, target, limits):
    """
    Creates an InputParameter object for Ruckig.

    Args:
        current (float): The current position.
        target (float): The target position.
        limits (dict): A dictionary containing 'v' (max velocity),
                       'a' (max acceleration), and 'r' (max jerk).

    Returns:
        InputParameter: The configured Ruckig InputParameter object.
    """
    inp = InputParameter(1)
    inp.current_position = [current]
    inp.current_velocity = [0.0]
    inp.current_acceleration = [0.0]
    inp.target_position = [target]
    inp.target_velocity = [0.0]
    inp.target_acceleration = [0.0]
    inp.max_velocity = [limits['v']]
    inp.max_acceleration = [limits['a']]
    inp.max_jerk = [limits['r']]
    return inp

# ===== Function to plot profiles =====
def plot_trajectory(name, data, time_step, limits_dict):
    """
    Plots the position, velocity, acceleration, and jerk profiles for a given joint.

    Args:
        name (str): The name of the joint (e.g., 'phi0', 'dh3').
        data (dict): A dictionary containing lists of 'pos', 'vel', and 'acc' data.
        time_step (float): The time step used in the simulation.
        limits_dict (dict): A dictionary containing the limits ('s', 'v', 'a', 'r')
                            for the current joint.
    """
    # Professional style configuration
    limits_dict = axes[name]

    plt.style.use('seaborn-v0_8-whitegrid')

    # Configure fonts (optional, requires LaTeX installed)
    rcParams['font.family'] = 'serif'
    rcParams['font.serif'] = ['Times New Roman']
    rcParams['mathtext.fontset'] = 'cm'

    # Data
    pos = np.array(data['pos'])
    vel = np.array(data['vel'])
    acc = np.array(data['acc'])
    t = np.arange(len(pos)) * time_step
    jerk = np.diff(acc, prepend=acc[0]) / time_step

    # Professional color palette (gray/blue tones)
    colors = {
        'jerk': '#4E5B6C',  # Dark bluish gray
        'acc': '#6C7B8B',    # Slate gray
        'vel': '#3A5FCD',    # Medium blue
        'pos': '#1E3F66',    # Dark blue
        'grid': '#E0E0E0',   # Very light gray
        'zero': '#606060'    # Medium gray
    }

    labels = [
        ('Jerk', jerk, r'$\dddot{q}(t)$ [rad/s$^3$ or m/s$^3$]', colors['jerk']),
        ('Acceleration', acc, r'$\ddot{q}(t)$ [rad/s$^2$ or m/s$^2$]', colors['acc']),
        ('Velocity', vel, r'$\dot{q}(t)$ [rad/s or m/s]', colors['vel']),
        ('Position', pos, r'$q(t)$ [rad or m]', colors['pos']),
    ]

    fig = plt.figure(figsize=(8, 10))
    plt.suptitle(f'Trajectory Analysis: {name}', y=0.98,
                 fontsize=14, fontweight='bold')

    for i, (title, data_i, ylabel, color) in enumerate(labels, 1):
        ax = plt.subplot(4, 1, i)

        # === Upper/lower limits based on type ===
        lim_key = {'Jerk': 'r', 'Acceleration': 'a', 'Velocity': 'v', 'Position': 's'}[title]
        if lim_key in limits_dict:
            limit = limits_dict[lim_key]
            ax.axhline(+limit, color='gray', linestyle='--', linewidth=1.2, alpha=0.8, label='Upper limit')
            ax.axhline(-limit, color='#5B7DB1', linestyle='--', linewidth=1.2, alpha=0.8, label='Lower limit')

        # Add right axis in degrees only for position and if it's 'phi'
        if title == 'Position' and 'phi' in name:
            def rad2deg(y): return np.rad2deg(y)
            def deg2rad(y): return np.deg2rad(y)
            ax_deg = ax.secondary_yaxis('right', functions=(rad2deg, deg2rad))
            ax_deg.set_ylabel(r'$q(t)$ [°]', fontsize=10)
            ax_deg.tick_params(axis='y', labelsize=9)

            # Force visible limits and ticks in degrees
            deg_min = np.floor(np.rad2deg(np.min(data_i)) / 30) * 30
            deg_max = np.ceil(np.rad2deg(np.max(data_i)) / 30) * 30
            ax_deg.set_yticks(np.arange(deg_min, deg_max + 1, 30))

            print(f"Adding secondary axis for {name}")

        # Thicker main line
        ax.plot(t, data_i, label=title, color=color,
                linewidth=1.8, alpha=0.9)
        ax.legend(loc='lower left', bbox_to_anchor=(0.4, 0.08), fontsize=6, frameon=False, ncol=2)

        ax.set_ylabel(ylabel, fontsize=10)
        ax.axhline(0, color=colors['zero'], linestyle=':',
                   linewidth=0.8, alpha=0.7)

        # More subtle grid
        ax.grid(True, color=colors['grid'], linestyle='--',
               linewidth=0.5, alpha=0.7)

        # Axis style
        for spine in ax.spines.values():
            spine.set_edgecolor(colors['grid'])
            spine.set_linewidth(0.8)

        if i == 4:
            ax.set_xlabel('Time [s]', fontsize=10)

        # Adjust ticks
        ax.tick_params(axis='both', which='both', labelsize=9,
                       direction='in', width=0.8)

        # Add title to each subplot (optional)
        ax.text(0.98, 0.92, title, transform=ax.transAxes,
                fontsize=10, ha='right', va='top',
                bbox=dict(facecolor='white', alpha=0.7,
                          edgecolor='none', pad=2))

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    # Save in high quality
    plt.savefig(output_folder / f"{name}_trajectory.png",
                dpi=600, bbox_inches='tight')
    plt.savefig(output_folder / f"{name}_trajectory.pdf",
                bbox_inches='tight')
    plt.close()

def plot_trajectory(name, data, time_step, limits_dict):
    """
    Plots the position, velocity, acceleration, and jerk profiles for a given joint.
    """
    # Professional configuration
    plt.style.use('default')
    
    # Configure fonts and text
    rcParams['font.family'] = 'serif'
    rcParams['font.serif'] = ['Times New Roman']
    rcParams['mathtext.fontset'] = 'cm'
    rcParams['axes.titlesize'] = 14
    rcParams['axes.labelsize'] = 12

    # Prepare data
    pos = np.array(data['pos'])
    vel = np.array(data['vel'])
    acc = np.array(data['acc'])
    t = np.arange(len(pos)) * time_step
    jerk = np.diff(acc, prepend=acc[0]) / time_step

    # Dictionary to convert names to LaTeX
    name_to_latex = {
        'phi0': r'$\varphi_0$',
        'phi1': r'$\varphi_1$',
        'phi2': r'$\varphi_2$',
        'dh3': r'$d_{h3}$',
        'phi4': r'$\varphi_4$',
        # Add more mappings as needed
    }
    
    # Get LaTeX representation or use default
    latex_name = name_to_latex.get(name, f'${name}$')

    # Professional subplot titles in English
    subplot_titles = {
        'Position': f'Position Profile: {latex_name}',
        'Velocity': f'Velocity Profile: {latex_name}',
        'Acceleration': f'Acceleration Profile: {latex_name}',
        'Jerk': f'Jerk Profile: {latex_name}'
    }

    # Professional color scheme (blues, grays, dark colors)
    colors = {
        'jerk': '#4C4C4C',    # Dark gray
        'acc': '#6D6D6D',      # Medium gray
        'vel': '#005073',      # Dark blue
        'pos': '#0080A3',      # Medium blue
        'limit': '#A2142F',    # Dark red for limits
        'grid': '#E0E0E0',     # Light gray for grid
        'zero': '#808080'      # Medium gray for zero line
    }

    # Create figure
    fig = plt.figure(figsize=(8, 10), dpi=100)
    plt.suptitle(f'Trajectory Analysis: {latex_name}', y=0.98,
                 fontsize=14, fontweight='bold')

    # Plot configurations for each subplot
    plot_configs = [
        ('Jerk', jerk, r'$\dddot{q}(t)$ [rad/s$^3$ or m/s$^3$]', colors['jerk']),
        ('Acceleration', acc, r'$\ddot{q}(t)$ [rad/s$^2$ or m/s$^2$]', colors['acc']),
        ('Velocity', vel, r'$\dot{q}(t)$ [rad/s or m/s]', colors['vel']),
        ('Position', pos, r'$q(t)$ [rad or m]', colors['pos'])
    ]

    for i, (title, data_i, ylabel, color) in enumerate(plot_configs, 1):
        ax = plt.subplot(4, 1, i)
        
        # Configure limits
        lim_key = {'Jerk': 'r', 'Acceleration': 'a', 'Velocity': 'v', 'Position': 's'}[title]
        if lim_key in limits_dict:
            limit = limits_dict[lim_key]
            ax.axhline(+limit, color=colors['limit'], linestyle='--', 
                      linewidth=1.5, alpha=0.8, label='Limit')
            ax.axhline(-limit, color=colors['limit'], linestyle='--', 
                      linewidth=1.5, alpha=0.8)

        # Plot data
        ax.plot(t, data_i, color=color, linewidth=2.0, alpha=0.9)
        
        # Configure secondary axis for angles
        if title == 'Position' and 'phi' in name:
            ax_deg = ax.secondary_yaxis('right', functions=(np.rad2deg, np.deg2rad))
            ax_deg.set_ylabel(r'$q(t)$ [°]', fontsize=10)

        # Styling
        ax.set_ylabel(ylabel, fontsize=10)
        ax.axhline(0, color=colors['zero'], linestyle=':', linewidth=1.0)
        ax.grid(True, color=colors['grid'], linestyle='-', linewidth=0.5)
        
        # Subplot title
        ax.set_title(subplot_titles[title], fontsize=11, pad=10, 
                    bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'))

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # Save in high quality
    output_folder.mkdir(exist_ok=True)
    plt.savefig(output_folder / f"{name}_trajectory.png",
               dpi=600, bbox_inches='tight', facecolor='white')
    plt.savefig(output_folder / f"{name}_trajectory.pdf",
               bbox_inches='tight', facecolor='white')
    plt.close()

if __name__ == "__main__":
    # ===== Joint trajectory simulation =====
    trajectories = {name: {'pos': [], 'vel': [], 'acc': []} for name in axes}
    timestamps = {name: [] for name in axes}

    for name, points in waypoints.items():
        current = 0.0
        for target in points[1:]:
            inp = create_input(current, target, axes[name])
            out = OutputParameter(1)
            time = 0.0
            while True:
                res = otg.update(inp, out)
                out.pass_to_input(inp)

                trajectories[name]['pos'].append(out.new_position[0])
                trajectories[name]['vel'].append(out.new_velocity[0])
                trajectories[name]['acc'].append(out.new_acceleration[0])
                timestamps[name].append(time)
                time += time_step

                if res == Result.Finished:
                    break
            current = target
        plot_trajectory(name, trajectories[name], time_step, axes[name])

    # ===== Save to .mat file =====
    savemat("ruckig_trajectories.mat", {'trajectories': trajectories})
    print("Trajectories exported to 'ruckig_trajectories.mat' and plots saved in /plots")
