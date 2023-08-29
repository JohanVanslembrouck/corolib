///////////////////////////////////////////////////////////////////////////////
// Copyright (c) Lewis Baker
// Licenced under MIT license. See LICENSE.txt for details.
///////////////////////////////////////////////////////////////////////////////
#ifndef CPPCORO_TESTS_TEMP_DIR_FIXTURE_HPP_INCLUDED
#define CPPCORO_TESTS_TEMP_DIR_FIXTURE_HPP_INCLUDED

#include "io_service_fixture.hpp"

namespace fs = std::filesystem;

namespace
{
    class temp_dir_fixture
    {
    public:

        temp_dir_fixture()
        {
            auto tempDir = fs::temp_directory_path();

            std::random_device random;
            for (int attempt = 1;; ++attempt)
            {
                m_path = tempDir / std::to_string(random());
                try
                {
                    fs::create_directories(m_path);
                    return;
                }
                catch (const fs::filesystem_error&)
                {
                    if (attempt == 10)
                    {
                        throw;
                    }
                }
            }
        }

        ~temp_dir_fixture()
        {
            fs::remove_all(m_path);
        }

        const std::filesystem::path& temp_dir()
        {
            return m_path;
        }

    private:

        std::filesystem::path m_path;

    };

    class temp_dir_with_io_service_fixture :
        public io_service_fixture,
        public temp_dir_fixture
    {
    };
}

#endif
