.PHONY: build-release
build-release:
	cmake -B build-release -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	cp ./build-release/compile_commands.json .
