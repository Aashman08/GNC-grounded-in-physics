"""Entry point for running the X-tail AUV simulation."""

from __future__ import annotations

from auv_sim import (
    XTailSimulationConfig,
    plot_longitudinal_response,
    simulate_xtail_six_dof,
)


def main() -> None:
    """Run the X-tail simulation and store artifacts."""

    print("Starting X-tail simulation...")
    config = XTailSimulationConfig()
    result = simulate_xtail_six_dof(config)

    states = result.states
    theta = states[:, 10]  # read pitch angle for plotting

    plot_path = plot_longitudinal_response(
        time=result.time,
        theta_command=result.theta_command,
        theta=theta,
        elevator=result.fin_deflections[:, 0],  # Plot first fin deflection
        save_dir=result.run_directory,
    )

    print(f"\n✓ X-tail simulation complete!")
    print(f"  Artifacts saved to: {result.run_directory}")
    print(f"  Longitudinal response: {plot_path}")
    print(f"\nFinal state:")
    print(f"  Position (x,y,z): ({states[-1,6]:.2f}, {states[-1,7]:.2f}, {states[-1,8]:.2f}) m")
    print(f"  Attitude (φ,θ,ψ): ({states[-1,9]:.2f}, {states[-1,10]:.2f}, {states[-1,11]:.2f}) rad")


if __name__ == "__main__":
    main()

