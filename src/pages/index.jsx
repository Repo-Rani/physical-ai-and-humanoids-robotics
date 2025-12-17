import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';


import Banner from '../components/banner';
import Hero from '../components/hero';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header>
      <div >
        <Heading as="h1" >
          <Hero/> 
      
        </Heading>
        
      </div>
    </header>
  );
}

export default function Home(){
  const {siteConfig} = useDocusaurusContext();
  return (
    <>
    <Banner/>
    <Layout
      title="AI & Humanoid Robotics"
      description="Ai & Humanoid Robotics TextBook">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
    </>
  );
}
