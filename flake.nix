{
  description = "OMNeT++ dev shell with GCC and Clang";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";
    flake-utils.url = "github:numtide/flake-utils";

    omnetpp = {
      url = "path:/home/bless/omnet/omnetpp-6.1";
    };

  };

  outputs = { self, nixpkgs, flake-utils, omnetpp }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        # Get the devShell from the OMNeT++ flake
        omnetShell = omnetpp.devShells.${system}.default;

        # Additional packages
        extraPackages = with pkgs; [
          #gcc
          #gccStdenv
          clang-tools
          clang
          boost
          gmp
        ];
      in {
        devShells.default = pkgs.mkShell {

        inputsFrom = [ omnetShell ];

        buildInputs = extraPackages;

        shellHook = ''
                      export PATH=$PATH:/home/bless/omnet/omnetpp-6.1/bin
                      echo "OMNeT++ dev shell with GCC and Clang ready."
                      echo "Using G++: $(g++ --version | head -n1)"
          '';
        };
      });
}
