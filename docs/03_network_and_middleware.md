# Networking & Middleware Configuration

## RMW Implementation
*   **Middleware:** Eclipse CycloneDDS (`rmw_cyclonedds_cpp`).

## WSL2 Networking Constraints
*   System uses split networking: RPi 5 on local network, RViz running inside WSL2.
*   WSL2 Hyper-V virtual switch blocks standard UDP multicast discovery.
*   **Agent Requirement:** Output a `cyclonedds.xml` configuration defining explicit unicast peers (`<Peer address="..."/>`) connecting the RPi 5 IP and WSL2 host IP. Include instructions to export `CYCLONEDDS_URI` in both `.bashrc` files.
