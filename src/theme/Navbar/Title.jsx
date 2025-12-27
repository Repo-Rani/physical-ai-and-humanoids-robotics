import React from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';

const NavbarTitle = () => {
  const { colorMode } = useColorMode();
  const isDarkTheme = colorMode === 'dark';

  return (
    <Link className="navbar__brand" to={useBaseUrl('/')}>
      <div className="text-xl sm:text-2xl font-extrabold ">
        <span
          className={`
            bg-clip-text
            text-transparent
            bg-gradient-to-r
            ${isDarkTheme
              ? 'from-green-400 via-emerald-500 to-teal-400'
              : 'from-green-600 via-emerald-600 to-teal-600'
            }
            animate-gradient-x
            font-extrabold
            tracking-tight
            whitespace-nowrap
            overflow-hidden
            transition-all duration-300
          `}
        >
          Humanoid Robotics
        </span>
      </div>
    </Link>
  );
};

export default NavbarTitle;
