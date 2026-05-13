# main.py
# ─────────────────────────────────────────────────────────────────────────────
# Entry point – just instantiate and run.
# ─────────────────────────────────────────────────────────────────────────────

from simulator import ContainerSimulator


def main():
    sim = ContainerSimulator()
    sim.run()


if __name__ == "__main__":
    main()
