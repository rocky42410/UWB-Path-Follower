#!/usr/bin/env python3
"""
Data Analysis for UWB Path Follower Logs
Generates figures for academic paper on cross-modal anomaly detection
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import signal, stats
from pathlib import Path
import argparse

# Set publication-quality defaults
plt.rcParams['figure.dpi'] = 150
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['font.size'] = 10
plt.rcParams['font.family'] = 'serif'
plt.rcParams['axes.labelsize'] = 11
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['xtick.labelsize'] = 9
plt.rcParams['ytick.labelsize'] = 9
plt.rcParams['legend.fontsize'] = 9
plt.rcParams['figure.figsize'] = (7, 5)

class UWBLogAnalyzer:
    def __init__(self, log_file):
        """Load and preprocess UWB path follower log."""
        self.df = pd.read_csv(log_file)
        self.preprocess()
        
    def preprocess(self):
        """Convert phases to numeric, compute derived metrics."""
        # Phase mapping
        phase_map = {'FWD': 0, 'SIT1': 1, 'TURN': 2, 'BACK': 3, 'SIT2': 4}
        if 'phase' in self.df.columns:
            self.df['phase_num'] = self.df['phase'].map(lambda x: phase_map.get(x, -1))
        
        # Compute position error magnitude
        if all(c in self.df.columns for c in ['x_est', 'y_est', 'x_pred', 'y_pred']):
            self.df['pos_error'] = np.sqrt(
                (self.df['x_est'] - self.df['x_pred'])**2 + 
                (self.df['y_est'] - self.df['y_pred'])**2
            )
        
        # Compute innovation magnitude
        if all(c in self.df.columns for c in ['innov_x', 'innov_y']):
            self.df['innov_mag'] = np.sqrt(
                self.df['innov_x']**2 + self.df['innov_y']**2
            )
        
        # Time relative to start
        if 'timestamp' in self.df.columns:
            self.df['time'] = self.df['timestamp'] - self.df['timestamp'].iloc[0]
    
    def plot_trajectory_comparison(self, save_path=None):
        """Figure 1: Trajectory with and without UWB fusion."""
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        
        # Left: Full trajectory
        ax = axes[0]
        ax.plot(self.df['x_pred'], self.df['y_pred'], 
                'r--', alpha=0.5, label='Dead Reckoning', linewidth=1)
        ax.plot(self.df['x_est'], self.df['y_est'], 
                'b-', alpha=0.8, label='UWB Fused', linewidth=1.5)
        
        # Mark waypoints
        ax.scatter([0, 5], [0, 0], c='green', s=100, marker='s', 
                  zorder=5, label='Waypoints')
        
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title('Path Following Performance')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        # Right: Position error over time
        ax = axes[1]
        ax.plot(self.df['time'], self.df['pos_error'] * 1000, 
                'k-', linewidth=1)
        ax.fill_between(self.df['time'], 0, self.df['pos_error'] * 1000, 
                        alpha=0.3)
        
        # Shade phases
        for phase in self.df['phase_num'].unique():
            mask = self.df['phase_num'] == phase
            if mask.any():
                t_start = self.df.loc[mask, 'time'].iloc[0]
                t_end = self.df.loc[mask, 'time'].iloc[-1]
                color = ['blue', 'gray', 'yellow', 'blue', 'gray'][int(phase)]
                ax.axvspan(t_start, t_end, alpha=0.1, color=color)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position Error (mm)')
        ax.set_title('Estimation Error Over Time')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, bbox_inches='tight')
        return fig
    
    def plot_innovation_analysis(self, save_path=None):
        """Figure 2: Innovation statistics and gating."""
        fig, axes = plt.subplots(2, 2, figsize=(10, 8))
        
        # Top left: Innovation time series
        ax = axes[0, 0]
        ax.plot(self.df['time'], self.df['innov_x'], 
                'r-', alpha=0.6, label='X Innovation', linewidth=0.8)
        ax.plot(self.df['time'], self.df['innov_y'], 
                'b-', alpha=0.6, label='Y Innovation', linewidth=0.8)
        ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        
        # Mark clipped innovations
        clipped = self.df[self.df['innov_clipped'] == True]
        if not clipped.empty:
            ax.scatter(clipped['time'], clipped['innov_x'], 
                      c='red', s=20, marker='x', label='Clipped')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Innovation (m)')
        ax.set_title('Position Innovations')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Top right: Innovation histogram
        ax = axes[0, 1]
        valid_innov = self.df[self.df['innov_ok'] == True]
        
        ax.hist(valid_innov['innov_mag'], bins=30, alpha=0.7, 
                color='blue', edgecolor='black')
        ax.axvline(x=0.5, color='r', linestyle='--', 
                  label='Gate Threshold')
        
        ax.set_xlabel('Innovation Magnitude (m)')
        ax.set_ylabel('Count')
        ax.set_title('Innovation Distribution')
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y')
        
        # Bottom left: Yaw innovation
        ax = axes[1, 0]
        ax.plot(self.df['time'], np.degrees(self.df['innov_yaw']), 
                'g-', alpha=0.7, linewidth=0.8)
        ax.fill_between(self.df['time'], 0, np.degrees(self.df['innov_yaw']), 
                        alpha=0.3, color='green')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Yaw Innovation (deg)')
        ax.set_title('Heading Corrections')
        ax.grid(True, alpha=0.3)
        
        # Bottom right: Gate acceptance rate
        ax = axes[1, 1]
        window = 50  # 1 second at 50Hz
        accept_rate = self.df['innov_ok'].rolling(window).mean() * 100
        
        ax.plot(self.df['time'], accept_rate, 'k-', linewidth=1.5)
        ax.fill_between(self.df['time'], 0, accept_rate, alpha=0.3)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceptance Rate (%)')
        ax.set_title('Innovation Gate Performance')
        ax.set_ylim([0, 105])
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, bbox_inches='tight')
        return fig
    
    def plot_cross_modal_correlation(self, save_path=None):
        """Figure 3: Cross-modal correlation matrix for invariants."""
        # Select relevant features
        features = ['cmd_vx', 'cmd_vy', 'cmd_wz', 
                   'gyro_z', 'roll', 'pitch',
                   'uwb_r', 'uwb_beta', 'uwb_alpha', 'uwb_gamma',
                   'innov_x', 'innov_y', 'innov_yaw']
        
        # Filter to available columns
        available = [f for f in features if f in self.df.columns]
        
        # Compute correlation matrix
        corr_matrix = self.df[available].corr()
        
        # Create figure
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Plot heatmap
        sns.heatmap(corr_matrix, annot=True, fmt='.2f', 
                   cmap='coolwarm', center=0, 
                   vmin=-1, vmax=1,
                   square=True, linewidths=0.5,
                   cbar_kws={"shrink": 0.8})
        
        ax.set_title('Cross-Modal Feature Correlations')
        
        # Rotate labels
        ax.set_xticklabels(ax.get_xticklabels(), rotation=45, ha='right')
        ax.set_yticklabels(ax.get_yticklabels(), rotation=0)
        
        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, bbox_inches='tight')
        return fig
    
    def plot_uwb_spherical_analysis(self, save_path=None):
        """Figure 4: UWB spherical measurements analysis."""
        fig, axes = plt.subplots(2, 2, figsize=(10, 8))
        
        # Range measurements
        ax = axes[0, 0]
        ax.plot(self.df['time'], self.df['uwb_r'], 'b-', linewidth=1)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Range (m)')
        ax.set_title('UWB Range Measurements')
        ax.grid(True, alpha=0.3)
        
        # Azimuth (beta)
        ax = axes[0, 1]
        ax.plot(self.df['time'], np.degrees(self.df['uwb_beta']), 
                'r-', linewidth=1)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Azimuth (deg)')
        ax.set_title('UWB Azimuth Angle')
        ax.grid(True, alpha=0.3)
        
        # Quality score
        ax = axes[1, 0]
        ax.plot(self.df['time'], self.df['uwb_quality'], 
                'g-', linewidth=1)
        ax.axhline(y=0.7, color='r', linestyle='--', 
                  label='Min Threshold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Quality Score')
        ax.set_title('UWB Measurement Quality')
        ax.set_ylim([0, 1.05])
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Polar plot of position
        ax = axes[1, 1]
        ax.remove()
        ax = plt.subplot(2, 2, 4, projection='polar')
        
        # Convert to polar coordinates
        r = self.df['uwb_r']
        theta = self.df['uwb_beta']
        
        # Color by phase
        scatter = ax.scatter(theta, r, c=self.df['phase_num'], 
                           cmap='viridis', s=1, alpha=0.5)
        
        ax.set_title('UWB Polar View')
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        
        plt.colorbar(scatter, ax=ax, label='Phase')
        
        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, bbox_inches='tight')
        return fig
    
    def plot_control_performance(self, save_path=None):
        """Figure 5: Control commands and filtering."""
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        
        # Forward velocity
        ax = axes[0]
        ax.plot(self.df['time'], self.df['cmd_vx'], 
                'b-', linewidth=1.5, label='Commanded')
        ax.axhline(y=0.3, color='k', linestyle='--', alpha=0.5, 
                  label='Nominal')
        ax.set_ylabel('Forward Velocity (m/s)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Lateral velocity (shows low-pass effect)
        ax = axes[1]
        ax.plot(self.df['time'], self.df['cmd_vy'], 
                'r-', linewidth=1, label='Filtered')
        ax.axhline(y=0.1, color='k', linestyle='--', alpha=0.5, 
                  label='Saturation')
        ax.axhline(y=-0.1, color='k', linestyle='--', alpha=0.5)
        ax.set_ylabel('Lateral Velocity (m/s)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Yaw rate
        ax = axes[2]
        ax.plot(self.df['time'], self.df['cmd_wz'], 
                'g-', linewidth=1, label='Filtered')
        ax.axhline(y=0.5, color='k', linestyle='--', alpha=0.5, 
                  label='Saturation')
        ax.axhline(y=-0.5, color='k', linestyle='--', alpha=0.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Yaw Rate (rad/s)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Shade phases
        for ax in axes:
            for phase in self.df['phase_num'].unique():
                mask = self.df['phase_num'] == phase
                if mask.any():
                    t_start = self.df.loc[mask, 'time'].iloc[0]
                    t_end = self.df.loc[mask, 'time'].iloc[-1]
                    color = ['blue', 'gray', 'yellow', 'blue', 'gray'][int(phase)]
                    ax.axvspan(t_start, t_end, alpha=0.1, color=color)
        
        plt.suptitle('Control Command Analysis')
        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, bbox_inches='tight')
        return fig
    
    def compute_anomaly_metrics(self):
        """Compute metrics for anomaly detection evaluation."""
        metrics = {}
        
        # Innovation statistics
        metrics['mean_innov_mag'] = self.df['innov_mag'].mean()
        metrics['std_innov_mag'] = self.df['innov_mag'].std()
        metrics['max_innov_mag'] = self.df['innov_mag'].max()
        
        # Gate performance
        metrics['gate_accept_rate'] = self.df['innov_ok'].mean()
        metrics['clip_rate'] = self.df['innov_clipped'].mean()
        
        # Position accuracy
        metrics['mean_pos_error'] = self.df['pos_error'].mean()
        metrics['max_pos_error'] = self.df['pos_error'].max()
        metrics['rmse'] = np.sqrt((self.df['pos_error']**2).mean())
        
        # UWB quality
        metrics['mean_uwb_quality'] = self.df['uwb_quality'].mean()
        metrics['uwb_dropout_rate'] = (self.df['uwb_quality'] < 0.7).mean()
        
        # Control smoothness (derivative of commands)
        if len(self.df) > 1:
            dt = self.df['time'].diff().mean()
            metrics['cmd_jerk_vx'] = np.diff(self.df['cmd_vx']).std() / dt
            metrics['cmd_jerk_vy'] = np.diff(self.df['cmd_vy']).std() / dt
            metrics['cmd_jerk_wz'] = np.diff(self.df['cmd_wz']).std() / dt
        
        return metrics
    
    def export_features_for_roca(self, output_file):
        """Export processed features for RoCA training."""
        # Select features for anomaly detection
        roca_features = [
            'cmd_vx', 'cmd_vy', 'cmd_wz',
            'gyro_z', 'roll', 'pitch',
            'foot_force_0', 'foot_force_1', 'foot_force_2', 'foot_force_3',
            'innov_mag', 'pos_error',
            'uwb_quality', 'zupt_active'
        ]
        
        # Filter available features
        available = [f for f in roca_features if f in self.df.columns]
        
        # Create feature matrix
        feature_df = self.df[available].copy()
        
        # Add derived features
        if 'cmd_vx' in feature_df.columns:
            feature_df['speed'] = np.sqrt(
                feature_df['cmd_vx']**2 + feature_df['cmd_vy']**2
            )
        
        # Add phase encoding
        if 'phase_num' in self.df.columns:
            feature_df['phase'] = self.df['phase_num']
        
        # Save to CSV
        feature_df.to_csv(output_file, index=False)
        print(f"Exported {len(feature_df)} samples with {len(feature_df.columns)} features to {output_file}")
        
        return feature_df

def main():
    parser = argparse.ArgumentParser(description='Analyze UWB path follower logs')
    parser.add_argument('log_file', help='Path to CSV log file')
    parser.add_argument('--output-dir', default='./figures', 
                       help='Directory for output figures')
    parser.add_argument('--export-features', action='store_true',
                       help='Export features for RoCA training')
    
    args = parser.parse_args()
    
    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Initialize analyzer
    print(f"Loading log file: {args.log_file}")
    analyzer = UWBLogAnalyzer(args.log_file)
    
    # Generate figures
    print("Generating figures...")
    
    analyzer.plot_trajectory_comparison(
        output_dir / 'fig1_trajectory.pdf'
    )
    print("  ✓ Figure 1: Trajectory comparison")
    
    analyzer.plot_innovation_analysis(
        output_dir / 'fig2_innovations.pdf'
    )
    print("  ✓ Figure 2: Innovation analysis")
    
    analyzer.plot_cross_modal_correlation(
        output_dir / 'fig3_correlations.pdf'
    )
    print("  ✓ Figure 3: Cross-modal correlations")
    
    analyzer.plot_uwb_spherical_analysis(
        output_dir / 'fig4_uwb_spherical.pdf'
    )
    print("  ✓ Figure 4: UWB spherical analysis")
    
    analyzer.plot_control_performance(
        output_dir / 'fig5_control.pdf'
    )
    print("  ✓ Figure 5: Control performance")
    
    # Compute metrics
    print("\nPerformance Metrics:")
    metrics = analyzer.compute_anomaly_metrics()
    for key, value in metrics.items():
        print(f"  {key}: {value:.4f}")
    
    # Export features if requested
    if args.export_features:
        feature_file = output_dir / 'roca_features.csv'
        analyzer.export_features_for_roca(feature_file)
    
    print(f"\nAll figures saved to {output_dir}")
    plt.show()

if __name__ == '__main__':
    main()