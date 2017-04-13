function(ENFORCE_OPTION_DEP_3DTK option VALUE)
  set (${option} "${VALUE}" CACHE BOOL "${${option}_DESCRIPTION}" FORCE) # this option set to VALUE as advised

  #now make sure other dependencies are also true
  foreach(d ${${option}_DEPENDENCIES}) # look through all my dependencies
    string(REGEX REPLACE " +" ";" CMAKE_DEPENDENT_OPTION_DEP "${d}")

    # check for a not in front
    string(STRIP "${CMAKE_DEPENDENT_OPTION_DEP}" CMAKE_DEPENDENT_OPTION_DEP)
    string(SUBSTRING "${CMAKE_DEPENDENT_OPTION_DEP}" 0 3 CMAKE_DEPENDENT_OPTION_DEP_3)
    string(TOUPPER "${CMAKE_DEPENDENT_OPTION_DEP_3}" CMAKE_DEPENDENT_OPTION_DEP_3)
    string(COMPARE EQUAL "${CMAKE_DEPENDENT_OPTION_DEP_3}" "NOT" CMAKE_DEPENDENT_OPTION_DEP_NOT)
    #string(REPLACE "NOT " "" CMAKE_DEPENDENT_OPTION_DEP "${d}")
    if(CMAKE_DEPENDENT_OPTION_DEP_NOT) # we found a NOT
      string(REPLACE "NOT;" "" CMAKE_DEPENDENT_OPTION_DEP "${CMAKE_DEPENDENT_OPTION_DEP}")
      if(${CMAKE_DEPENDENT_OPTION_DEP})  # not met, make sure it is
        ENFORCE_OPTION_DEP_3DTK(${CMAKE_DEPENDENT_OPTION_DEP} OFF)
      else() # dependency is met
      endif()
    else()
      if(${CMAKE_DEPENDENT_OPTION_DEP})  # dependency is met
      else() # not met, make sure it is
        ENFORCE_OPTION_DEP_3DTK(${CMAKE_DEPENDENT_OPTION_DEP} ON)
      endif()
    endif()
  endforeach()

endfunction()

macro(OPT_DEP option doc default depends)
  option(${option} "${doc}" "${default}")
  set(${option} "${${option}}" CACHE BOOL "${doc}" FORCE)
  set(${option}_DEPENDENCIES "${depends}" CACHE INTERNAL "" FORCE)
  set(${option}_DESCRIPTION "${doc}" CACHE INTERNAL "" FORCE)

  if (${option})
    #message(STATUS "Yes ${option} is true")
    #  message("FOREACH d in ${depends}")

    foreach(d ${depends})
      string(REGEX REPLACE " +" ";" CMAKE_DEPENDENT_OPTION_DEP "${d}")

      # check for a not in front
      string(STRIP "${CMAKE_DEPENDENT_OPTION_DEP}" CMAKE_DEPENDENT_OPTION_DEP)
      string(SUBSTRING "${CMAKE_DEPENDENT_OPTION_DEP}" 0 3 CMAKE_DEPENDENT_OPTION_DEP_3)
      string(TOUPPER "${CMAKE_DEPENDENT_OPTION_DEP_3}" CMAKE_DEPENDENT_OPTION_DEP_3)
      string(COMPARE EQUAL "${CMAKE_DEPENDENT_OPTION_DEP_3}" "NOT" CMAKE_DEPENDENT_OPTION_DEP_NOT)
      if(CMAKE_DEPENDENT_OPTION_DEP_NOT) # we found a NOT
        string(REPLACE "NOT;" "" CMAKE_DEPENDENT_OPTION_DEP "${CMAKE_DEPENDENT_OPTION_DEP}")
        if(${CMAKE_DEPENDENT_OPTION_DEP})  # not met, make sure it is
          ENFORCE_OPTION_DEP_3DTK(${CMAKE_DEPENDENT_OPTION_DEP} OFF)
        else() # dependency is met
        endif()
      else()
        if(${CMAKE_DEPENDENT_OPTION_DEP})  # dependency is met
        else() # not met, make sure it is
          ENFORCE_OPTION_DEP_3DTK(${CMAKE_DEPENDENT_OPTION_DEP} ON)
        endif()
      endif()
    endforeach()

  endif()
endmacro()
