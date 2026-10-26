#pragma once

#include <iostream>
#include <stdexcept>
#include <system_error>
#include "1_Platform/compat/filesystem.hpp"

namespace vista::transport::detail {
// Shared by an owner and its immutable disk readers. No directory sweep: this
// lease can delete only the exact child it exclusively created in this process.
struct SessionDirectory {
    vista::fs::path root, canonical_root;
    bool owned{}, discard{};
    void ensure() {
        if(owned) return;
        vista::fs::create_directories(root.parent_path());
        const auto parent=vista::fs::canonical(root.parent_path());
        if(!vista::fs::create_directory(root))
            throw std::runtime_error("map session cache already exists; refusing reuse");
        canonical_root=vista::fs::canonical(root);
        if(canonical_root.parent_path()!=parent || canonical_root.filename()!=root.filename())
            throw std::runtime_error("map session cache escaped its parent");
        owned=true;
    }
    ~SessionDirectory() noexcept {
        if(!owned || !discard) return;
        try {
            if(!vista::fs::exists(root)) return;
            if(vista::fs::is_symlink(root) || vista::fs::canonical(root)!=canonical_root ||
               canonical_root==canonical_root.root_path() || canonical_root.filename().empty()) {
                std::cerr<<"Map cache cleanup skipped: path changed: "<<root.string()<<'\n';
                return;
            }
            std::error_code error;
            vista::fs::remove_all(canonical_root,error);
            if(error) std::cerr<<"Map cache cleanup warning: "<<error.message()<<" at "<<root.string()<<'\n';
        } catch(const std::exception& error) {
            std::cerr<<"Map cache cleanup warning: "<<error.what()<<'\n';
        }
    }
};
} // namespace vista::transport::detail
