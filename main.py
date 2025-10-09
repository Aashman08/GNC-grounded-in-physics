"""Entry point for running the modular AUV simulation."""

from __future__ import annotations

from auv_sim import (
    SimulationConfig,
    plot_longitudinal_response,
    simulate_six_dof,
)


def main() -> None:
    """Run the simulation and store artifacts under the conventioned folder."""

    config = SimulationConfig()
    result = simulate_six_dof(config)

    states = result.states
    theta = states[:, 10]  # read pitch angle for plotting

    plot_path = plot_longitudinal_response(
        time=result.time,
        theta_command=result.theta_command,
        theta=theta,
        elevator=result.elevator,
        save_dir=result.run_directory,
    )

    print(f"Simulation complete. Artifacts saved to {result.run_directory}")
    print(f"Longitudinal response figure: {plot_path}")


if __name__ == "__main__":
    main()