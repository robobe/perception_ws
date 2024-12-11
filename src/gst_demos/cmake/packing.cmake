
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

set(PACKAGE_XML "${CMAKE_CURRENT_SOURCE_DIR}/package.xml")


# extract name description email
function(metadata)
    file(READ "${PACKAGE_XML}" PACKAGE_XML_CONTENTS)
    # name
    string(REGEX MATCH "<name>([^<]+)<\/name>" NAME_TAG "${PACKAGE_XML_CONTENTS}")
    string(REGEX REPLACE "<name>([^<]+)</name>" "\\1" NAME "${NAME_TAG}")
    set(CPACK_PACKAGE_NAME "${NAME}" PARENT_SCOPE)

    # description
    string(REGEX MATCH "<description>([^<]+)<\/description>" DESC_TAG "${PACKAGE_XML_CONTENTS}")
    string(REGEX REPLACE "<description>([^<]+)</description>" "\\1" DESC "${DESC_TAG}")
    set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "${DESC}" PARENT_SCOPE)

    # maintainer email
    string(REGEX MATCH "<maintainer email=\"([^\"]+)\">" EMAIL_MATCH "${PACKAGE_XML_CONTENTS}")
    string(REGEX REPLACE ".*email=\"([^\"]+)\".*" "\\1" EXTRACTED_EMAIL "${EMAIL_MATCH}")
    set(CPACK_PACKAGE_CONTACT "${EXTRACTED_EMAIL}" PARENT_SCOPE)

    
    # maintainer name
    string(REGEX MATCH "<maintainer email=\"[^\"]+\">([^<]+)<\/maintainer>" MATCH "${PACKAGE_XML_CONTENTS}")
    string(REGEX REPLACE ".*<maintainer email=\"[^\"]+\">([^<]+)<\/maintainer>.*" "\\1" EXTRACTED_TEXT "${MATCH}")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "${EXTRACTED_TEXT}" PARENT_SCOPE)

endfunction()

# Function to parse dependencies from package.xml
function(parse_package_dependencies)
    file(READ "${PACKAGE_XML}" PACKAGE_XML_CONTENTS)

    # Match <exec_depend> tags, handling whitespace
    string(REGEX MATCHALL "<exec_depend>([^<]+)<\/exec_depend>" DEPENDENCY_MATCHES "${PACKAGE_XML_CONTENTS}")

    set(DEBIAN_DEPENDS "")

    foreach(DEPEND ${DEPENDENCY_MATCHES})
        string(REGEX REPLACE "<exec_depend>([^<]+)</exec_depend>" "\\1" CLEANED "${DEPEND}")
        list(APPEND DEBIAN_DEPENDS "${CLEANED}")
    endforeach()

    string(JOIN "," MY_DEBIAN_DEPENDS ${DEBIAN_DEPENDS})
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "${MY_DEBIAN_DEPENDS}" PARENT_SCOPE)
endfunction()

function(parse_version)
    file(READ "${PACKAGE_XML}" PACKAGE_XML_CONTENTS)
    # string(REGEX MATCH "<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>" VERSION_MATCH "${PACKAGE_XML_CONTENTS}")
    string(REGEX REPLACE ".*<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>.*" "\\1" CPACK_PACKAGE_VERSION_MAJOR "${PACKAGE_XML_CONTENTS}")
    string(REGEX REPLACE ".*<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>.*" "\\2" CPACK_PACKAGE_VERSION_MINOR "${PACKAGE_XML_CONTENTS}")
    string(REGEX REPLACE ".*<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>.*" "\\3" CPACK_PACKAGE_VERSION_PATCH "${PACKAGE_XML_CONTENTS}")

    set(CPACK_PACKAGE_VERSION_MAJOR "${CPACK_PACKAGE_VERSION_MAJOR}" PARENT_SCOPE)
    set(CPACK_PACKAGE_VERSION_MINOR "${CPACK_PACKAGE_VERSION_MINOR}" PARENT_SCOPE)
    set(CPACK_PACKAGE_VERSION_PATCH "${CPACK_PACKAGE_VERSION_PATCH}" PARENT_SCOPE)
endfunction()

parse_package_dependencies()
parse_version()
metadata()

include(CPack)
