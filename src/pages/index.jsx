import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import BrowserOnly from '@docusaurus/BrowserOnly';

import Banner from '../components/banner';
import Hero from '../components/hero';

function HomepageHeader() {
  return (
    <header>
      <div>
        <Heading as="h1">
          <BrowserOnly>
            {() => <Hero />}
          </BrowserOnly>
        </Heading>
      </div>
    </header>
  );
}

export default function Home() {
  return (
    <>
      <BrowserOnly>
        {() => <Banner />}
      </BrowserOnly>

      <Layout
        title="AI & Humanoid Robotics"
        description="AI & Humanoid Robotics TextBook">
        
        <HomepageHeader />
        
        <main>
          <HomepageFeatures />
        </main>
      </Layout>
    </>
  );
}
