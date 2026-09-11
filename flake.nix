{
  description = "EdgeTX firmware development environment";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  # EdgeTX enforces this exact compiler release in radio/src/CMakeLists.txt.
  inputs.nixpkgs-toolchain.url = "github:NixOS/nixpkgs/nixos-25.05";

  outputs = { nixpkgs, nixpkgs-toolchain, ... }:
    let
      supportedSystems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
      forAllSystems = nixpkgs.lib.genAttrs supportedSystems;
    in
    {
      devShells = forAllSystems (
        system:
        let
          pkgs = import nixpkgs { inherit system; };
          toolchainPkgs = import nixpkgs-toolchain { inherit system; };
          python = pkgs.python3.withPackages (pythonPackages: with pythonPackages; [
            asciitree
            jinja2
            libclang
            lz4
            pillow
            pyelftools
            tkinter
          ]);
        in
        {
          default = pkgs.mkShell {
            packages = with pkgs; [
              cmake
              dfu-util
              file
              gawk
              git
              gnumake
              python
              toolchainPkgs.gcc-arm-embedded-14
              unzip
              zip
            ];

            shellHook = ''
              echo "EdgeTX development shell"
              echo "GNU Arm Embedded: $(arm-none-eabi-gcc --version | head -n 1)"
              echo "Configure MODAL_ZORRO from a build directory with:"
              echo "  cmake -DPCB=M207 -DPCBREV=MODAL_ZORRO -DDEFAULT_MODE=2 -DINTERNAL_MODULE_MULTI=NO -DMULTIMODULE=NO -DGHOST=NO -DDEBUG_SEGGER_RTT=n -DDEBUG=n -DCMAKE_BUILD_TYPE=RELEASE .."
              echo "Configure the HELM_BASIC release image from the repository root with:"
              echo "  cmake -S . -B build-modal-helm -DPCB=M196 -DPCBREV=HELM_BASIC -DDEFAULT_MODE=2 -DINTERNAL_MODULE_MULTI=NO -DMULTIMODULE=NO -DGHOST=NO -DDEBUG_SEGGER_RTT=OFF -DDEBUG=OFF -DHELM_DIAGNOSTICS=OFF -DCMAKE_BUILD_TYPE=Release"
              echo "Configure the HELM_BASIC RTT debug image from the repository root with:"
              echo "  cmake -S . -B build-modal-helm-rtt -DPCB=M196 -DPCBREV=HELM_BASIC -DDEFAULT_MODE=2 -DINTERNAL_MODULE_MULTI=NO -DMULTIMODULE=NO -DGHOST=NO -DHELM_DIAGNOSTICS=ON -DDEBUG=RTT -DDEBUG_SEGGER_RTT=ON -DENABLE_BOOTLOADER_DEBUG=OFF -DDEBUG_SEGGER_SYSVIEW=OFF -DTEST_BUILD_WARNING=ON -DWATCHDOG=ON -DOPT=s -DCMAKE_BUILD_TYPE=Debug"
              echo "See the M0196 port document for build and J-Link commands."
            '';
          };
        }
      );
    };
}
