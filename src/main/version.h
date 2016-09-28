/*
 * This file is part of RaceFlight.
 *
 * RaceFlight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RaceFlight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RaceFlight.  If not, see <http://www.gnu.org/licenses/>.
 */

#define FC_VERSION_TYPE_AUTO "BB"
#define FC_BUILD_NUMBER_AUTO "345"

#define	FC_VERSION_IDENT			"RF"
#define FC_VERSION_MAJOR            1
#define FC_VERSION_MINOR            9
#define FC_VERSION_PATCH_LEVEL      9

#define FC_VERSION_COMMENT           "Increment version"

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define FC_VERSION_STRING			STR(FC_VERSION_IDENT) " " STR(FC_VERSION_MAJOR) "." STR(FC_VERSION_MINOR) "." STR(FC_VERSION_PATCH_LEVEL) " " STR(FC_VERSION_TYPE_AUTO) STR(FC_BUILD_NUMBER_AUTO)

#define MW_VERSION              234

extern const char* const targetName;

#define GIT_SHORT_REVISION_LENGTH   7
extern const char* const shortGitRevision;

#define BUILD_DATE_LENGTH 11
extern const char* const buildDate;

#define BUILD_TIME_LENGTH 8
extern const char* const buildTime;
